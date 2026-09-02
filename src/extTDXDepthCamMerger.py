"""

	TDXDepthCamMerger (DBLib)
	____________________________

	Darien Brito
	info@darienbrito.com
	https://www.darienbrito.com

	Lines up the point clouds of several depth cameras into one shared space.
	The registration maths is Open3D's, all credit for it goes to its authors.
	http://www.open3d.org/

	Open3D is never imported here: loading it inside TouchDesigner crashes the
	app. It runs in a separate python instead (see the workerSource DAT) and
	sends back a 4x4 matrix. So this component does not care which python
	TouchDesigner itself uses.

	Everywhere below a pair is [target, source]. The target stays where it is,
	the source is the cloud moved onto it.
"""

import hashlib
import json
import os
import subprocess
import tempfile

import numpy as np

# Also defined in extUtilities: the two DATs are separate modules inside TD
# and cannot import each other.
DEVICE_PREFIX = 'Device'
WORKER_TIMEOUT = 900

MATRIX_COLUMNS = [f'm{r}{c}' for r in range(4) for c in range(4)]
CALIBRATION_COLUMNS = ['device', 'parent', 'method', 'fitness', 'rmse',
	'correspondences', 'status'] + MATRIX_COLUMNS


def deviceName(index):
	"""The COMP name for a device index: Device1, Device2, ..."""
	return f'{DEVICE_PREFIX}{int(index)}'


# ________________________________________________________________ pure maths


def matrixFromValues(values):
	"""16 numbers, read row by row, into a 4x4."""
	numbers = [float(v) for v in values]
	if len(numbers) != 16:
		raise ValueError(f'expected 16 matrix values, got {len(numbers)}')
	return np.array(numbers, dtype=np.float64).reshape(4, 4)


def looksRigid(matrix, tolerance=0.1):
	"""Quick check that a typed in matrix is only a move and a turn."""
	bottom = np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-4)
	scale = abs(abs(np.linalg.det(matrix[:3, :3])) - 1.0) < tolerance
	return bool(bottom and scale)


def composeChain(links, reference=1):
	"""
	Turn camera to camera transforms into transforms onto one reference camera.

	links: {device: (parent, matrix)}, where matrix moves that device into its
	parent's space. Returns {device: matrix} into the reference space.

	The parent's matrix goes on the LEFT of the child's. The other way round
	applies the parent's motion in the child's space, which is wrong.
	"""
	reference = int(reference)
	composed = {reference: np.eye(4)}

	def resolve(device, seen):
		device = int(device)
		if device in composed:
			return composed[device]
		if device in seen:
			raise ValueError(f'device chain loops back on device {device}')
		if device not in links:
			raise ValueError(
				f'device {device} has no calibration and is not the reference')
		seen.add(device)
		parent, matrix = links[device]
		composed[device] = resolve(parent, seen) @ np.asarray(matrix, dtype=np.float64)
		return composed[device]

	for device in list(links):
		resolve(device, set())
	return composed


# _________________________________________________________________ extension


class extTDXDepthCamMerger:
	"""Lines up point clouds from several depth cameras."""

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
		Write the worker DAT out as a real .py file so another python can run
		it. The file name holds a hash of the text, so an edited DAT gets a new
		file and an old one is never reused.
		"""
		dat = self.ownerComp.op('workerSource')
		if dat is None:
			raise ValueError(f'no workerSource DAT in {self.ownerComp.path}')
		source = dat.text
		digest = hashlib.sha1(source.encode('utf-8')).hexdigest()[:12]
		path = os.path.join(self.workDir(), f'worker_{digest}.py')
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
			raise ValueError(f'Python exe does not exist: {exe}')
		return exe

	def resultPath(self):
		return os.path.join(self.workDir(), 'result.json')

	def runWorker(self, args, timeout=WORKER_TIMEOUT, resultPath=None):
		"""
		Run the worker and return its JSON reply. Raises on any failure.

		The result file is deleted before the run, so a worker that dies
		cannot hand back the previous run's answer.
		"""
		if resultPath and os.path.isfile(resultPath):
			os.remove(resultPath)
		command = [self.pythonExe(), self.workerPath()] + list(args)
		try:
			proc = subprocess.run(command, capture_output=True, text=True,
				timeout=timeout, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
		except subprocess.TimeoutExpired:
			raise RuntimeError(f'worker timed out after {timeout}s')

		payload = self.workerReply(proc, resultPath)
		if payload is None:
			raise RuntimeError('worker produced no result.\nstdout: {}\nstderr: {}'.format(
				(proc.stdout or '')[-500:], (proc.stderr or '')[-500:]))
		if not payload.get('ok'):
			raise RuntimeError('worker failed: {}'.format(payload.get('error')))
		return payload

	def workerReply(self, proc, resultPath):
		"""
		The worker's parsed JSON reply, or None when it left none.

		The reply is read from a file rather than from what the worker
		printed, because open3d prints to stdout as well.
		"""
		payload = None
		if resultPath and os.path.isfile(resultPath):
			try:
				with open(resultPath, encoding='utf-8') as handle:
					payload = json.load(handle)
			except ValueError:
				payload = None
		if payload is None:
			# No file to read: --probe writes none, and a worker that died may
			# not have got that far. Take the last line it printed instead.
			for line in reversed((proc.stdout or '').strip().splitlines()):
				try:
					payload = json.loads(line)
					break
				except ValueError:
					continue
		return payload

	def findDevice(self, index):
		"""The device COMP for this index, or None when it does not exist."""
		return self.ownerComp.op(deviceName(index))

	def device(self, index):
		comp = self.findDevice(index)
		if comp is None:
			raise ValueError(f'no {deviceName(index)} inside {self.ownerComp.path}')
		return comp

	def dumpCloud(self, index, withColors, tag):
		"""
		Copy a device's cloud off the GPU into files the worker can read.

		The cloud is taken before the transform and before Show, so a hidden
		device can still be calibrated. Pixels the camera never returned are
		already gone, and every colour sits on the point it belongs to.
		"""
		comp = self.device(index)
		pop = comp.op('null_sourcePointcloud')
		if pop is None:
			raise ValueError(f'{comp.path} has no null_sourcePointcloud')
		if not pop.numPoints():
			raise ValueError(f'{pop.path} has no points. Either its source resolves '
				'to nothing, or the mask threw everything away.')

		points = np.array(list(pop.points('P')), dtype=np.float32)
		pointPath = os.path.join(self.workDir(), f'{tag}_points.npy')
		np.save(pointPath, points)

		colorPath = None
		if withColors:
			# A "custom" device may have no colour source, and its points then
			# carry a default colour. Say so rather than matching flat grey.
			if 'Color' not in [a.name for a in pop.pointAttributes]:
				raise ValueError(f'{pop.path} carries no Color attribute, which '
					'coloured ICP needs. Give this device a colour source or turn '
					'off Usecoloricp.')
			colors = np.array(list(pop.points('Color')), dtype=np.float32)[:, :3]
			colorPath = os.path.join(self.workDir(), f'{tag}_colors.npy')
			np.save(colorPath, colors)
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
			raise ValueError(f'no calibrationData table in {self.ownerComp.path}')
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
		"""{device: (parent, matrix)} read out of the calibration table."""
		table = self.calibrationTable()
		links = {}
		for index in range(1, table.numRows):
			device = int(table[index, 'device'].val)
			parent = int(table[index, 'parent'].val)
			links[device] = (parent,
				matrixFromValues([table[index, name].val for name in MATRIX_COLUMNS]))
		return links

	def writeMatrix(self, table, matrix):
		"""Write a 4x4 as four table rows, the layout the transform POP reads."""
		table.clear()
		for row in np.asarray(matrix, dtype=np.float64):
			table.appendRow([repr(float(v)) for v in row])

	def matrixFromDat(self):
		"""
		Use a 4x4 made elsewhere (CloudCompare, OpenCV, a hand alignment) from a
		table DAT as this pair's transform.
		"""
		table = self.par('Presetmatrixdat', None)
		if table is None:
			raise ValueError('Preset matrix DAT is not set')
		if table.numRows != 4 or table.numCols != 4:
			raise ValueError(f'{table.path} is {table.numRows}x{table.numCols}, '
				'expected a 4x4')
		try:
			matrix = matrixFromValues([c.val for row in table.rows() for c in row])
		except ValueError as err:
			raise ValueError(f'{table.path}: {err}')

		status = 'OK' if looksRigid(matrix) else 'WARN'
		if status == 'WARN':
			print(f'[TDXMerger] {table.path} does not look like a rigid transform '
				'(bottom row or determinant is off)')
		return matrix, {'stage': 'table', 'fitness': 1.0, 'rmse': 0.0,
			'correspondences': 0, 'status': status}

	def report(self, result):
		self.ownerComp.par.Lastfitness = '{:.4f}'.format(result.get('fitness', 0.0))
		self.ownerComp.par.Lastrmse = '{:.5f}'.format(result.get('rmse', 0.0))
		self.ownerComp.par.Lastcorrespondences = int(result.get('correspondences', 0))
		parts = ['{}: {} fitness {:.3f}, rmse {:.4f} m over {} points'.format(
			result.get('status', '?'), result.get('stage', '?'),
			result.get('fitness', 0.0), result.get('rmse', 0.0),
			result.get('correspondences', 0))]
		first = result.get('global')
		if first:
			parts.append('(global stage: fitness {:.3f}, rmse {:.4f} m)'.format(
				first.get('fitness', 0.0), first.get('rmse', 0.0)))
		# Fitness only says the solver settled. Whether repeated tries landed in
		# the same place is what tracks a right answer, so it goes in the line
		# the user reads.
		if not result.get('agreed', True):
			parts.append('UNSTABLE: repeated runs landed {:.2f} m apart, so check '
				'the viewport before trusting this'.format(result.get('consensus', 0.0)))
		message = '   '.join(parts)
		self.ownerComp.par.Laststatus = message
		print(f'[TDXMerger] {message}')
		# No popup for an unstable result. It fires often enough that people
		# would learn to click it away, so the status line carries it instead.
		if result.get('status') == 'FAIL':
			ui.messageBox('Calibration failed', (
				'{}\n\nThe matrix was still written so you can inspect it, but the '
				'two views probably do not overlap enough, or the scene is too '
				'featureless for feature matching.').format(message))

	# ____ Public ____

	def CheckWorker(self):
		"""Check that the outside python can really run the registration."""
		try:
			info = self.runWorker(['--probe'], timeout=120)
		except Exception as err:
			self.ownerComp.par.Open3dstatus = f'FAILED: {err}'
			self.ownerComp.par.Open3dversion = ''
			self.ownerComp.par.Pythonversion = ''
			print(f'[TDXMerger] {err}')
			return False
		# The status field is the verdict, the versions sit beside it. numpy only
		# matters when something is broken, so it goes to the console.
		self.ownerComp.par.Open3dstatus = 'OK'
		self.ownerComp.par.Open3dversion = str(info.get('open3d') or '')
		self.ownerComp.par.Pythonversion = str(info.get('python') or '')
		print('[TDXMerger] worker OK: open3d {} on python {} (numpy {})'.format(
			info.get('open3d'), info.get('python'), info.get('numpy')))
		return True

	def Calibrate(self, pair=(1, 2), mode='globalRegistration'):
		"""
		Work out the transform that puts the source cloud onto the target.

		Mode 'globalThenIcp' does the rough match and the refine in one worker
		run, which is what calibrating from scratch wants: one process start and
		one read of each cloud. 'globalRegistration' stops after the rough match
		so you can look at it before refining.
		"""
		target, source = int(pair[0]), int(pair[1])
		if target == source:
			raise ValueError(f'target and source are both device {target}')

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
		Improve an existing calibration with ICP. It never starts a fresh rough
		match on its own: Refine only polishes what is already there.
		"""
		target, source = int(pair[0]), int(pair[1])
		links = self.readLinks()
		if source not in links:
			raise ValueError(
				f'no calibration stored for device {source}. Run Calibrate first.')
		parent, matrix = links[source]
		if parent != target:
			raise ValueError(
				f'device {source} is calibrated against device {parent}, not '
				f'{target}. Recalibrate, or set the pair to ({parent}, {source}).')

		result = self.runWorker([self.buildJob((target, source), 'icp', init=matrix)],
			resultPath=self.resultPath())
		refined = matrixFromValues(result['matrix'])
		self.storeCalibration(source, parent, refined, result)
		self.report(result)
		self.RebuildChain()
		return refined

	def RebuildChain(self):
		"""
		Redo every device's transform onto the reference camera and write them
		to the tables the GPU reads. Cheap enough to redo in full.
		"""
		reference = int(self.par('Referencedevice', 1))
		composed = composeChain(self.readLinks(), reference)
		for index in range(1, int(self.par('Numberofdevices', 1)) + 1):
			# findDevice, not device(): a partly built rig is fine here, a
			# missing COMP is simply skipped.
			comp = self.findDevice(index)
			if comp is None:
				continue
			table = comp.op('transformMatrix')
			if table is not None:
				self.writeMatrix(table, composed.get(index, np.eye(4)))
		return composed

	def ResetCalibration(self):
		"""Forget every calibration and leave all devices untransformed."""
		table = self.calibrationTable()
		table.clear()
		table.appendRow(CALIBRATION_COLUMNS)
		self.RebuildChain()
