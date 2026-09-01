"""

	TDXDepthCamMerger (DBLib)
	____________________________

	Darien Brito
	info@darienbrito.com
	https://www.darienbrito.com

	Registers point clouds from several depth cameras into one coordinate space.
	The registration itself is Open3D's; all credit for those algorithms goes to
	its authors. http://www.open3d.org/

	Open3D is NOT imported here. Importing it inside TouchDesigner hard-crashes
	the process, because Open3D ships its own OpenMP/TBB alongside the ones
	TouchDesigner already loaded. Instead the maths runs in a separate python
	process (see the workerSource DAT) and only a 4x4 matrix comes back. That
	also means this component does not care which python TouchDesigner itself
	uses, so it works on builds whose python has no Open3D wheel at all.

	Convention throughout: pair = [target, source]. The target is the reference
	and does not move; the source is the cloud transformed onto it.
"""

import hashlib
import json
import os
import subprocess
import tempfile

import numpy as np

DEVICE_PREFIX = 'Device'
WORKER_TIMEOUT = 900

MATRIX_COLUMNS = ['m{}{}'.format(r, c) for r in range(4) for c in range(4)]
CALIBRATION_COLUMNS = ['device', 'parent', 'method', 'fitness', 'rmse',
	'correspondences', 'status'] + MATRIX_COLUMNS


# ________________________________________________________________ pure maths


def matrixFromValues(values):
	"""16 values in row major order into a 4x4."""
	numbers = [float(v) for v in values]
	if len(numbers) != 16:
		raise ValueError('expected 16 matrix values, got {}'.format(len(numbers)))
	return np.array(numbers, dtype=np.float64).reshape(4, 4)


def looksRigid(matrix, tolerance=0.1):
	"""Cheap sanity check on a matrix a user typed or pasted in."""
	bottom = np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-4)
	scale = abs(abs(np.linalg.det(matrix[:3, :3])) - 1.0) < tolerance
	return bool(bottom and scale)


def composeChain(links, reference=1):
	"""
	Compose pairwise transforms into the reference device's frame.

	links: {device: (parent, matrix)}, matrix mapping that device's frame into
	its parent's frame. Returns {device: matrix} into the reference frame.

	Order is load bearing. Open3D returns T with x_target = T @ x_source, so the
	parent's composed matrix multiplies on the LEFT of the child's pairwise one.
	The other way round applies the parent's motion in the child's frame, which
	is the bug this replaces: before, each device applied only its own pairwise
	matrix, so anything past the second camera landed in the wrong place.
	"""
	reference = int(reference)
	composed = {reference: np.eye(4)}

	def resolve(device, seen):
		device = int(device)
		if device in composed:
			return composed[device]
		if device in seen:
			raise ValueError('device chain loops back on device {}'.format(device))
		if device not in links:
			raise ValueError(
				'device {} has no calibration and is not the reference'.format(device))
		seen.add(device)
		parent, matrix = links[device]
		composed[device] = resolve(parent, seen) @ np.asarray(matrix, dtype=np.float64)
		return composed[device]

	for device in list(links):
		resolve(device, set())
	return composed


# _________________________________________________________________ extension


class extTDXDepthCamMerger:
	"""Multi camera point cloud registration for TouchDesigner."""

	def __init__(self, ownerComp):
		self.ownerComp = ownerComp

	# ____ Private ____

	def par(self, name, default=None):
		parameter = getattr(self.ownerComp.par, name, None)
		return default if parameter is None else parameter.eval()

	def workDir(self):
		path = os.path.join(tempfile.gettempdir(), 'tdxmerger')
		os.makedirs(path, exist_ok=True)
		return path

	def workerPath(self):
		"""
		Materialise the embedded worker source as a real .py so a separate
		interpreter can run it. Named by content hash, so editing the DAT
		produces a new file and stale copies are never reused.
		"""
		dat = self.ownerComp.op('workerSource')
		if dat is None:
			raise ValueError('no workerSource DAT in {}'.format(self.ownerComp.path))
		source = dat.text
		digest = hashlib.sha1(source.encode('utf-8')).hexdigest()[:12]
		path = os.path.join(self.workDir(), 'worker_{}.py'.format(digest))
		if not os.path.isfile(path):
			with open(path, 'w', encoding='utf-8', newline='\n') as handle:
				handle.write(source)
		return path

	def pythonExe(self):
		exe = str(self.par('Pythonexe', '') or '').strip()
		if not exe:
			raise ValueError(
				'Python exe is not set. Point it at a python.exe (3.11 or newer) '
				'that has open3d installed, then pulse Check worker.')
		if not os.path.isfile(exe):
			raise ValueError('Python exe does not exist: {}'.format(exe))
		return exe

	def resultPath(self):
		return os.path.join(self.workDir(), 'result.json')

	def runWorker(self, args, timeout=WORKER_TIMEOUT, resultPath=None):
		"""
		Run the worker and return its parsed JSON. Raises on any failure.

		The worker writes its reply to resultPath as well as printing it. Prefer
		the file: open3d writes to stdout of its own accord, and picking the
		reply back out of a stream by trying json.loads on every line is the
		weaker of the two. The file is removed first, so a worker that dies
		before writing cannot hand back the previous run's answer.
		"""
		if resultPath and os.path.isfile(resultPath):
			os.remove(resultPath)
		command = [self.pythonExe(), self.workerPath()] + list(args)
		try:
			proc = subprocess.run(command, capture_output=True, text=True,
				timeout=timeout, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
		except subprocess.TimeoutExpired:
			raise RuntimeError('worker timed out after {}s'.format(timeout))

		payload = None
		if resultPath and os.path.isfile(resultPath):
			try:
				with open(resultPath, encoding='utf-8') as handle:
					payload = json.load(handle)
			except ValueError:
				payload = None
		if payload is None:
			# --probe writes no file, and a worker killed mid-write leaves none.
			for line in reversed((proc.stdout or '').strip().splitlines()):
				try:
					payload = json.loads(line)
					break
				except ValueError:
					continue
		if payload is None:
			raise RuntimeError('worker produced no result.\nstdout: {}\nstderr: {}'.format(
				(proc.stdout or '')[-500:], (proc.stderr or '')[-500:]))
		if not payload.get('ok'):
			raise RuntimeError('worker failed: {}'.format(payload.get('error')))
		return payload

	def device(self, index):
		comp = self.ownerComp.op('{}{}'.format(DEVICE_PREFIX, int(index)))
		if comp is None:
			raise ValueError('no {}{} inside {}'.format(
				DEVICE_PREFIX, int(index), self.ownerComp.path))
		return comp

	def dumpCloud(self, index, withColors, tag):
		"""
		Pull a device's cloud off the GPU and write it where the worker can read
		it. Points go out unfiltered; the worker drops the invalid ones so that
		points and colours are masked identically in one place.

		numpyArray() returns the image vertically flipped. That only permutes the
		order of an unordered point set and both clouds are read the same way, so
		the registration is unaffected. Do not "fix" it.
		"""
		comp = self.device(index)
		top = comp.op('null_sourcePointcloud')
		if top is None:
			raise ValueError('{} has no null_sourcePointcloud'.format(comp.path))

		array = top.numpyArray()
		points = array.reshape(-1, array.shape[-1])[:, :3].astype(np.float32)
		pointPath = os.path.join(self.workDir(), '{}_points.npy'.format(tag))
		np.save(pointPath, points)

		colorPath = None
		if withColors:
			colorTop = comp.op('null_color')
			if colorTop is None:
				raise ValueError(
					'{} has no null_color, which coloured ICP needs'.format(comp.path))
			# Reachable with device type "custom": a customSources row may leave
			# the colour cell empty, which leaves null_color with no input and
			# zero resolution. Say so here rather than shipping a bad array.
			if not (colorTop.width and colorTop.height):
				raise ValueError('{} resolves to no colour image, which coloured ICP '
					'needs. Give this device a colour source or turn off '
					'Usecoloricp.'.format(colorTop.path))
			colors = colorTop.numpyArray()
			colors = colors.reshape(-1, colors.shape[-1])[:, :3]
			if colors.dtype == np.uint8:
				colors = colors.astype(np.float32) / 255.0
			colorPath = os.path.join(self.workDir(), '{}_colors.npy'.format(tag))
			np.save(colorPath, colors.astype(np.float32))
		return pointPath, colorPath

	def buildJob(self, pair, mode, init=None):
		colored = bool(self.par('Usecoloricp', False)) and mode in ('icp', 'globalThenIcp')
		targetPoints, targetColors = self.dumpCloud(pair[0], colored, 'target')
		sourcePoints, sourceColors = self.dumpCloud(pair[1], colored, 'source')

		job = {
			'mode': mode,
			'target': targetPoints, 'source': sourcePoints,
			'targetColors': targetColors, 'sourceColors': sourceColors,
			'voxel': float(self.par('Voxelsize', 0.05)),
			'refineVoxel': float(self.par('Refinevoxel', 0.01)),
			'maxRange': float(self.par('Maxrange', 0.0)),
			'seed': int(self.par('Seed', -1)),
			'colored': colored,
			'init': None if init is None else [float(v) for v in np.asarray(init).reshape(-1)],
			'result': self.resultPath(),
		}
		path = os.path.join(self.workDir(), 'job.json')
		with open(path, 'w', encoding='utf-8') as handle:
			json.dump(job, handle)
		return path

	# ____ Calibration store ____

	def calibrationTable(self):
		table = self.ownerComp.op('calibrationData')
		if table is None:
			raise ValueError('no calibrationData table in {}'.format(self.ownerComp.path))
		if table.numRows == 0 or [c.val for c in table.row(0)] != CALIBRATION_COLUMNS:
			table.clear()
			table.appendRow(CALIBRATION_COLUMNS)
		return table

	def storeCalibration(self, device, parent, matrix, result):
		table = self.calibrationTable()
		row = [str(int(device)), str(int(parent)), result.get('stage', '?'),
			'{:.6f}'.format(result.get('fitness', 0.0)),
			'{:.6f}'.format(result.get('rmse', 0.0)),
			str(result.get('correspondences', 0)), result.get('status', '?')]
		row += [repr(float(v)) for v in np.asarray(matrix, dtype=np.float64).reshape(-1)]

		existing = table.row(str(int(device)))
		if existing is None:
			table.appendRow(row)
		else:
			for index, value in enumerate(row):
				table[existing[0].row, index] = value

	def readLinks(self):
		"""{device: (parent, matrix)} from the calibration store."""
		table = self.calibrationTable()
		links = {}
		for index in range(1, table.numRows):
			device = int(table[index, 'device'].val)
			parent = int(table[index, 'parent'].val)
			links[device] = (parent,
				matrixFromValues([table[index, name].val for name in MATRIX_COLUMNS]))
		return links

	def writeMatrix(self, table, matrix):
		"""
		Write a 4x4 as four rows. Verified in TD 2025.33070: the GLSL matrix
		uniform reads a table DAT row-major, so this is the layout the shader
		expects. A +1 X translation written this way translates the cloud by 1 m.
		"""
		table.clear()
		for row in np.asarray(matrix, dtype=np.float64):
			table.appendRow([repr(float(v)) for v in row])

	def report(self, result):
		self.ownerComp.par.Lastfitness = '{:.4f}'.format(result.get('fitness', 0.0))
		self.ownerComp.par.Lastrmse = '{:.5f}'.format(result.get('rmse', 0.0))
		self.ownerComp.par.Lastcorrespondences = int(result.get('correspondences', 0))
		message = '{}: {} fitness {:.3f}, rmse {:.4f} m over {} points'.format(
			result.get('status', '?'), result.get('stage', '?'),
			result.get('fitness', 0.0), result.get('rmse', 0.0),
			result.get('correspondences', 0))
		first = result.get('global')
		if first:
			message += '   (global stage: fitness {:.3f}, rmse {:.4f} m)'.format(
				first.get('fitness', 0.0), first.get('rmse', 0.0))
		self.ownerComp.par.Laststatus = message
		print('[TDXMerger] {}'.format(message))
		if result.get('status') == 'FAIL':
			ui.messageBox('Calibration failed', (
				'{}\n\nThe matrix was still written so you can inspect it, but the '
				'two views probably do not overlap enough, or the scene is too '
				'featureless for feature matching.').format(message))

	# ____ Public ____

	def CheckWorker(self):
		"""Verify the external interpreter can actually run the registration."""
		try:
			info = self.runWorker(['--probe'], timeout=120)
		except Exception as err:
			self.ownerComp.par.Open3dstatus = 'FAILED: {}'.format(err)
			print('[TDXMerger] {}'.format(err))
			return False
		self.ownerComp.par.Open3dstatus = 'OK: open3d {} on python {} (numpy {})'.format(
			info.get('open3d'), info.get('python'), info.get('numpy'))
		self.ownerComp.par.Open3dversion = str(info.get('open3d') or '')
		self.ownerComp.par.Pythonversion = str(info.get('python') or '')
		return True

	def Calibrate(self, pair=(1, 2), mode='globalRegistration'):
		"""
		Estimate the transform bringing the source cloud onto the target.

		mode 'globalThenIcp' chains RANSAC into ICP inside one worker run, which
		is what calibrating from scratch actually wants: it costs one process
		start and one readback of each cloud instead of two. 'globalRegistration'
		keeps the global stage on its own for anyone who wants to look at it
		before refining.
		"""
		target, source = int(pair[0]), int(pair[1])
		if target == source:
			raise ValueError('target and source are both device {}'.format(target))

		if mode == 'table':
			matrix, result = self.matrixFromDat()
		else:
			stage = 'globalThenIcp' if mode == 'globalThenIcp' else 'global'
			result = self.runWorker([self.buildJob((target, source), stage)],
				resultPath=self.resultPath())
			matrix = matrixFromValues(result['matrix'])

		self.storeCalibration(source, target, matrix, result)
		self.report(result)
		self.RebuildChain()
		return matrix

	def Refine(self, pair=(1, 2)):
		"""
		Polish an existing calibration with ICP. Never silently falls back to a
		fresh global registration: Refine means improve what is already there.
		"""
		target, source = int(pair[0]), int(pair[1])
		links = self.readLinks()
		if source not in links:
			raise ValueError(
				'no calibration stored for device {}. Run Calibrate first.'.format(source))
		parent, matrix = links[source]
		if parent != target:
			raise ValueError(
				'device {} is calibrated against device {}, not {}. Recalibrate, or '
				'set the pair to ({}, {}).'.format(source, parent, target, parent, source))

		result = self.runWorker([self.buildJob((target, source), 'icp', init=matrix)],
			resultPath=self.resultPath())
		refined = matrixFromValues(result['matrix'])
		self.storeCalibration(source, parent, refined, result)
		self.report(result)
		self.RebuildChain()
		return refined

	def RebuildChain(self):
		"""
		Recompose every device's transform into the reference frame and push the
		result to the tables the GPU reads. Cheap enough to redo in full.
		"""
		reference = int(self.par('Referencedevice', 1))
		composed = composeChain(self.readLinks(), reference)
		for index in range(1, int(self.par('Numberofdevices', 1)) + 1):
			comp = self.ownerComp.op('{}{}'.format(DEVICE_PREFIX, index))
			if comp is None:
				continue
			table = comp.op('transformMatrix')
			if table is not None:
				self.writeMatrix(table, composed.get(index, np.eye(4)))
		return composed

	def ResetCalibration(self):
		"""Forget every stored calibration and put all devices back at identity."""
		table = self.calibrationTable()
		table.clear()
		table.appendRow(CALIBRATION_COLUMNS)
		self.RebuildChain()

	def matrixFromDat(self):
		"""
		Take a 4x4 produced elsewhere (CloudCompare, OpenCV, a manual alignment)
		from a table DAT and use it as the pairwise matrix.
		"""
		table = self.par('Presetmatrixdat', None)
		if table is None:
			raise ValueError('Preset matrix DAT is not set')
		if table.numRows != 4 or table.numCols != 4:
			raise ValueError('{} is {}x{}, expected a 4x4'.format(
				table.path, table.numRows, table.numCols))
		try:
			matrix = matrixFromValues([c.val for row in table.rows() for c in row])
		except ValueError as err:
			raise ValueError('{}: {}'.format(table.path, err))

		status = 'OK' if looksRigid(matrix) else 'WARN'
		if status == 'WARN':
			print('[TDXMerger] {} does not look like a rigid transform '
				'(bottom row or determinant is off)'.format(table.path))
		return matrix, {'stage': 'table', 'fitness': 1.0, 'rmse': 0.0,
			'correspondences': 0, 'status': status}
