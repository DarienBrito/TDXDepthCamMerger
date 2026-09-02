"""
	Finds the cameras and builds one COMP per camera.

	Knows nothing about Open3D. What each camera needs lives in the deviceTypes
	table, so a new camera is a new row, not new code.

	The clouds are POPs. Only the three source operators are TOPs, because that
	is what the camera SDKs give. Everything after them is points.
"""

# Also defined in extTDXDepthCamMerger: the two DATs are separate modules
# inside TD and cannot import each other.
DEVICE_PREFIX = 'Device'

# The source TOPs each device reads from, rebuilt per camera type.
SOURCE_NAMES = ('in_pointcloud', 'in_color', 'in_mask')

# Older source names. Deleted on sight so nothing stale is left behind.
LEGACY_SOURCE_NAMES = ('kinectazureselect1', 'kinectazureselect2', 'kinectazureselect3')

# Older chain names. Deleted on sight, so a rebuilt device does not keep
# cooking operators nothing reads any more.
LEGACY_CHAIN_NAMES = ('null_color', 'thresh1', 'multiply1', 'switch1',
	'glsl2', 'glsl2_pixel', 'glsl2_info', 'mtx')

# The point chain, in order. Rebuilt whole, because how many attributes it
# carries depends on whether the camera has a mask.
POP_NAMES = ('pop_convert', 'pop_valid', 'null_sourcePointcloud',
	'pop_transform', 'pop_show', 'null_pointCloud')

# Where each source TOP sits in the network.
SOURCE_POSITIONS = {
	'in_color': (-275, -275),
	'in_mask': (-275, -125),
	'in_pointcloud': (-50, 25),
}

# Where each chain node sits: sources on the left, then one row running left to
# right, with transformMatrix underneath it.
POP_POSITIONS = {
	'pop_convert': (150, 25),
	'pop_valid': (325, 25),
	'null_sourcePointcloud': (500, 25),
	'pop_transform': (675, 25),
	'pop_show': (850, 25),
	'null_pointCloud': (1025, 25),
}

# The Kinect writes 255 where it sees nobody and a small number where it sees a
# body, so a body pixel is at most 5/255.
MASK_THRESHOLD = 0.036

# Higher than the mask can ever reach, which switches masking off without
# changing the operator. The mask is a texture channel, so it never passes 1.
MASK_OFF = 1e9

# Attributes only used inside the chain. The merged output carries P and Color.
WORKING_ATTRIBUTES = ('valid', 'maskv')

# Radius of the sphere at the camera origin that pop_valid throws away, in
# metres. A depth camera writes an exact (0, 0, 0) where it got no return, and
# every camera has a minimum range in the tens of centimetres, so nothing real
# is ever this close to the sensor.
ORIGIN_RADIUS = 0.001


def deviceName(index):
	"""The COMP name for a device index: Device1, Device2, ..."""
	return f'{DEVICE_PREFIX}{int(index)}'


def destroyByName(comp, names):
	"""Destroy any child of comp going by one of these names."""
	for name in names:
		stale = comp.op(name)
		if stale is not None:
			stale.destroy()


def cameraTopExpr(shortcut, cameraOpName):
	"""
	Expression for a camera select: the nth camera TOP inside whatever the
	Inputs op parameter points at.
	"""
	return ("op(parent.{0}.par.Inputsop).op(f'{1}{{parent().digits}}')"
		" if parent.{0}.par.Inputsop != None else ''").format(shortcut, cameraOpName)


def customTopExpr(shortcut, column):
	"""
	Expression for a custom device: one customSources row per device.

	An empty mask cell falls back to the row's point cloud, so the mask input
	always has a TOP to read. That makes maskv equal valid, which the mask
	threshold then ignores. The fallback lives here, where it can be seen,
	rather than in CustomSource, which still answers "does this row have a
	mask" honestly.
	"""
	call = "parent.{}.CustomSource(parent().digits, '{{}}')".format(shortcut)
	if column == 'mask':
		return '{} or {}'.format(call.format('mask'), call.format('pointcloud'))
	return call.format(column)


class extUtilities:
	"""Housekeeping: find cameras, build one device COMP per camera."""

	def __init__(self, ownerComp):
		self.ownerComp = ownerComp

	# ____ Private ____

	def deviceTypeRow(self):
		"""The deviceTypes row matching the Device type parameter."""
		table = self.ownerComp.op('deviceTypes')
		if table is None:
			raise ValueError('no deviceTypes table in {}'.format(self.ownerComp.path))
		wanted = self.ownerComp.par.Devicetype.eval()
		row = table.row(wanted)
		if row is None:
			raise ValueError('deviceTypes has no row for "{}"'.format(wanted))
		return {table[0, c].val: row[c].val for c in range(table.numCols)}

	def sourceSpecs(self, spec):
		"""
		Which source TOPs this camera needs, as (name, image, column).

		A camera with no mask image gets no mask source at all. "custom" always
		gets one, because there the mask is set per device rather than by the
		camera type.
		"""
		specs = [
			('in_pointcloud', spec['image_pointcloud'], 'pointcloud'),
			('in_color', spec['image_color'], 'color'),
		]
		if spec['type'] == 'custom' or spec['image_mask']:
			specs.append(('in_mask', spec['image_mask'], 'mask'))
		return specs

	def maskThresholdExpr(self, spec):
		"""
		What drives the mask condition, or None when the camera has no mask.

		A point is kept while maskv is BELOW the threshold. Putting the
		threshold out of reach switches masking off without changing the
		operator, which matters because clones all have to look the same: one
		custom row may have a mask and the next may not, so that difference has
		to live in an expression.
		"""
		if spec['type'] == 'custom':
			return ("{} if (parent().par.Useplayer and parent.{}.CustomSource("
				"parent().digits, 'mask')) else {}".format(
					MASK_THRESHOLD, self.ownerComp.par.parentshortcut.eval(), MASK_OFF))
		if spec['image_mask']:
			return '{} if parent().par.Useplayer else {}'.format(MASK_THRESHOLD, MASK_OFF)
		return None

	def createConvert(self, device, masked):
		"""
		pop_convert: the toptoPOP that turns the source textures into points.

		Attributes are declared first: an input block can only write to an
		attribute that already exists. Color is built in, the other two are
		ours (WORKING_ATTRIBUTES).
		"""
		convert = device.create(toptoPOP, 'pop_convert')
		convert.cook(force=True)
		convert.par.rgba = 'custom'
		convert.par.surftype = 'points'
		# Cook around numBlocks: a sequence block's parameters do not exist
		# until the operator has cooked with the new block count.
		convert.seq.attr.numBlocks = 3 if masked else 2
		convert.cook(force=True)
		convert.par.attr0name = 'color'
		for index, name in enumerate(WORKING_ATTRIBUTES[:convert.seq.attr.numBlocks - 1]):
			convert.par[f'attr{index + 1}name'] = 'custom'
			convert.par[f'attr{index + 1}customname'] = name
			convert.par[f'attr{index + 1}type'] = 'float'
			convert.par[f'attr{index + 1}numcomps'] = '1'
			convert.par[f'attr{index + 1}defaultval0'] = 1.0

		# Channel names are separated by spaces ('r g b', not 'rgb'), and an
		# attribute needs a component ('valid.x'), or it silently writes zero.
		# Colour is taken as rgba because Color holds four values.
		blocks = [
			('in_pointcloud', 'r g b', 'P'),
			('in_color', 'r g b a', 'Color'),
			('in_pointcloud', 'a', 'valid.x'),
		]
		if masked:
			blocks.append(('in_mask', 'a', 'maskv.x'))
		convert.seq.input.numBlocks = len(blocks)
		convert.cook(force=True)
		for index, (source, channels, attribute) in enumerate(blocks):
			convert.par[f'input{index}top'] = source
			convert.par[f'input{index}chanscope'] = channels
			convert.par[f'input{index}attrscope'] = attribute
		return convert

	def createPointFilter(self, device, name, upstream, conditions, dropOrigin=False):
		"""
		A deletePOP set to KEEP the points that pass every condition.

		conditions: (attribute, function, value) rows, one sequence block
		each, ANDed together. A string value is wired in as an expression, a
		number is set as a constant.

		dropOrigin adds a bounding sphere at the origin and keeps what is
		OUTSIDE it. The Attribute and Bounding pages are ANDed, so it is a
		second, independent reason to throw a point away. See dropOriginBound.
		"""
		node = device.create(deletePOP, name)
		node.inputConnectors[0].connect(upstream)
		node.cook(force=True)
		node.par.entity = 'point'
		node.par.invert = 'keep'
		# Cook around numBlocks: a sequence block's parameters do not exist
		# until the operator has cooked with the new block count.
		node.seq.attr.numBlocks = len(conditions)
		node.cook(force=True)
		for index, (attribute, function, value) in enumerate(conditions):
			if index:
				node.par[f'attr{index}combine'] = 'and'
			node.par[f'attr{index}inattr'] = attribute
			node.par[f'attr{index}func'] = function
			if isinstance(value, str):
				node.par[f'attr{index}value'].expr = value
			else:
				node.par[f'attr{index}value'] = value
		if dropOrigin:
			self.dropOriginBound(node)
		return node

	def dropOriginBound(self, node):
		"""
		Keep only what is outside a small sphere at the camera origin.

		The alpha channel of a point cloud TOP is validity, and pop_convert
		reads it into `valid`. That is undocumented for both cameras TD ships,
		so a sensor that writes alpha 1 everywhere would put every unreturned
		pixel, all sitting at exactly (0, 0, 0), into the merged cloud. The
		geometry says the same thing the alpha does and no camera can disagree
		with it: this runs before pop_transform, so the origin is the sensor
		itself.

		Measured 2026-09-02 on a 49152 texel synthetic cloud: the alpha
		condition alone kept 24480, the sphere alone kept the same 24480, and
		with the alpha condition widened to keep all 49152 the sphere still
		left 24480, which is what proves the two pages are ANDed.
		"""
		node.seq.bound.numBlocks = 1
		node.cook(force=True)
		node.par.bound0enabled = True
		node.par.bound0inattr = 'P'
		node.par.bound0type = 'boundingsphere'
		for axis in 'xyz':
			node.par['bound0translate' + axis] = 0
			node.par['bound0scale' + axis] = ORIGIN_RADIUS
		node.par.bound0invert = True
		return node

	def buildPopChain(self, device, built, spec):
		"""
		Rebuild the device's point chain on top of freshly built source TOPs.

		pop_convert reads the three TOPs by PARAMETER, not by wire: a POP has no
		TOP input, so the sources sit beside the chain rather than before it.

			pop_convert  makes points out of the source textures
			pop_valid    drops the pixels the camera never returned, by alpha
			             and by position, and the masked out ones. Colours
			             travel with the points
			null_sourcePointcloud  this camera's own cloud, what Calibrate reads
			pop_transform  applies the calibration
			pop_show     Show, after the sample point, so hiding a device still
			             leaves it calibratable
			null_pointCloud  what World merges
		"""
		destroyByName(device, POP_NAMES + LEGACY_CHAIN_NAMES)

		convert = self.createConvert(device, masked='in_mask' in built)

		# Drop the pixels the camera never returned, and the masked out ones
		# when this camera has a mask at all (see maskThresholdExpr).
		conditions = [('valid.x', 'gte', 0.5)]
		threshold = self.maskThresholdExpr(spec)
		if threshold:
			conditions.append(('maskv.x', 'lt', threshold))
		keep = self.createPointFilter(device, 'pop_valid', convert, conditions,
			dropOrigin=True)

		source = device.create(nullPOP, 'null_sourcePointcloud')
		source.inputConnectors[0].connect(keep)

		# transformMatrix holds the calibration, one matrix row per table row.
		xform = device.create(transformPOP, 'pop_transform')
		xform.inputConnectors[0].connect(source)
		xform.par.xformmatrixop = 'transformMatrix'

		# Show hides a device from the merged cloud. It sits here, after the
		# point Calibrate samples, so a hidden device still has points to
		# register. Everything that got this far has valid 1, so a limit above
		# that keeps nothing.
		hide = self.createPointFilter(device, 'pop_show', xform,
			[('valid.x', 'gte', '0.5 if parent().par.Show else 2.0')])

		out = device.create(nullPOP, 'null_pointCloud')
		out.inputConnectors[0].connect(hide)

		for name, (x, y) in POP_POSITIONS.items():
			node = device.op(name)
			node.nodeX, node.nodeY = x, y
			# Syncing a clone does not carry the input and attribute blocks set
			# up above: they come back at their defaults, and the device then
			# keeps every pixel with nothing reporting a problem. Marking the
			# chain clone immune keeps what was copied. Nothing is lost, because
			# structural changes go through RebuildDevices, which copies the
			# template again.
			node.cloneImmune = True
		return {name: device.op(name) for name in POP_NAMES}

	def applyMaskAvailability(self, device, spec):
		"""
		Tell the UI whether masking is possible for this camera at all. Which
		branch a given device takes is the mask threshold, set in buildPopChain.
		"""
		me = self.ownerComp
		if spec['type'] == 'custom':
			table = me.op('customSources')
			available = table is not None and any(table[r, 'mask'].val.strip()
				for r in range(1, table.numRows))
		else:
			available = bool(spec['image_mask'])
		me.par.Usemaskforcalibration.enable = available
		# Inputs op is read by every device type, custom included, so it stays
		# enabled. See CustomSource.
		me.par.Inputsop.enable = True

	def recognizeDevices(self):
		"""
		Ask the camera TOP which cameras are attached. Which parameter holds
		that list differs per camera (kinectazure uses `sensor`, orbbec uses
		`device`), so it comes from the deviceTypes table.

		Returns None when the cameras were found, or a message saying what is
		wrong. It does NOT open the dialog itself: a message box freezes
		TouchDesigner until someone clicks it, which would hang any scripted
		run. GatherDevices decides how to report.
		"""
		me = self.ownerComp
		spec = self.deviceTypeRow()

		if spec['type'] == 'custom':
			table = me.op('customSources')
			count = max(0, table.numRows - 1) if table else 0
			if not count:
				return ('Device type is "custom" but the customSources table is '
					'empty. Add one row per device.')
			# A row whose point cloud cell resolves to nothing builds a device
			# that sees nothing and adds nothing to a calibration, silently. The
			# camera types already refuse in that case, so custom does too.
			dead = self.UnresolvedCustomRows()
			if dead:
				return ('These customSources rows do not resolve to a point cloud '
					'TOP:\n\n{}\n\nInputs op is {}. Cells are looked up inside it, '
					'so either point it at the base holding these TOPs or give the '
					'cells full paths.'.format(
						'\n'.join('  row {}: {} -> {!r}'.format(r, name, path)
							for r, name, path in dead),
						me.par.Inputsop.eval() or '(empty)'))
			me.par.Numberofdevices = count
			me.par.Devices = ','.join(
				table[r, 0].val for r in range(1, table.numRows))
			return None

		target = me.op(me.par.Inputsop)
		if target is None:
			return ('Inputs op does not point at anything. Set it to the base '
				'holding your camera TOPs.')

		first = target.op('{}1'.format(spec['cameraop_name']))
		if first is None:
			return 'Could not find a "{}1" TOP inside {}.'.format(
				spec['cameraop_name'], target.path)

		parameter = getattr(first.par, spec['devicepar'], None)
		if parameter is None:
			return '{} has no "{}" parameter.'.format(first.path, spec['devicepar'])

		names = [n for n in (parameter.menuNames or []) if n]
		if not names:
			return 'No cameras found. Is anything plugged in?'

		me.par.Numberofdevices = len(names)
		me.par.Devices = ','.join(names)
		return None

	def destroyDevices(self):
		"""Drop every device clone except the template, Device1."""
		for child in list(self.ownerComp.children):
			if not child.name.startswith(DEVICE_PREFIX):
				continue
			suffix = child.name[len(DEVICE_PREFIX):]
			if suffix.isdigit() and int(suffix) > 1:
				child.destroy()

	def createDevices(self):
		"""Clone the template once per extra camera."""
		me = self.ownerComp
		template = me.op(deviceName(1))
		if template is None:
			raise ValueError(f'no {deviceName(1)} template in {me.path}')
		for index in range(2, int(me.par.Numberofdevices) + 1):
			clone = me.copy(template, name=deviceName(index))
			clone.nodeX = template.nodeX
			clone.nodeY = template.nodeY - 200 * (index - 1)
			clone.par.clone = template.name

	# ____ Public ____

	def UnresolvedCustomRows(self):
		"""
		customSources rows whose point cloud cell resolves to nothing, as
		(row, name, cell). Kept apart from the warning that reports it, so a
		test can ask the question without a dialog opening.

		Only the point cloud is required. An empty mask cell is how a row says
		the device has no mask, and colour only matters for coloured ICP.
		"""
		table = self.ownerComp.op('customSources')
		if table is None:
			return []
		return [(r, table[r, 0].val, table[r, 'pointcloud'].val.strip())
			for r in range(1, table.numRows)
			if self.CustomSource(r, 'pointcloud') is None]

	def CustomSource(self, index, column):
		"""
		Resolve one customSources cell to an operator, or None.

		Called from every source TOP of a custom device, so one table row is one
		device and the row number is the device's trailing digit.

		Cells are looked up INSIDE the base that Inputs op points at, exactly
		like the camera types, so several sets of test TOPs can live in their
		own bases and be swapped by repointing that one parameter. With Inputs
		op empty they are looked up in the component instead, and a full path
		works either way.

		An empty mask cell answers None, which is what tells the chain this
		device has no mask. The fallback that keeps pop_convert fed lives in the
		in_mask expression, not here.
		"""
		table = self.ownerComp.op('customSources')
		if table is None or not 0 < index < table.numRows:
			return None
		path = table[index, column].val.strip()
		if not path:
			return None
		base = self.ownerComp.op(self.ownerComp.par.Inputsop) or self.ownerComp
		return base.op(path)

	def BuildDeviceSources(self, device=None):
		"""
		Rebuild a device's source TOPs and point chain from its deviceTypes row.

		This is what makes a new camera a table row instead of new code. Only
		the Device1 template needs it: clones copy the template, and each source
		finds its own camera through `parent().digits`.
		"""
		me = self.ownerComp
		if device is None:
			device = me.op(deviceName(1))
		if device is None:
			raise ValueError(f'no {deviceName(1)} template in {me.path}')

		spec = self.deviceTypeRow()
		shortcut = me.par.parentshortcut.eval()
		custom = spec['type'] == 'custom'

		destroyByName(device, SOURCE_NAMES + LEGACY_SOURCE_NAMES)

		built = {}
		for name, image, column in self.sourceSpecs(spec):
			try:
				top = device.create(spec['selectop'], name)
			except Exception as exc:
				raise ValueError('deviceTypes row "{}" names an operator type TD does '
					'not know: "{}" ({})'.format(spec['type'], spec['selectop'], exc))
			top.nodeX, top.nodeY = SOURCE_POSITIONS[name]
			top.par.top.expr = (customTopExpr(shortcut, column) if custom
				else cameraTopExpr(shortcut, spec['cameraop_name']))
			if image:
				top.par.image = image
			built[name] = top

		# Line the colour image up with the depth image. Only the Kinect select
		# can do it, so ask rather than assume.
		if hasattr(built['in_color'].par, 'remapimage'):
			built['in_color'].par.remapimage = True

		# A plain select TOP has no active switch, so for custom devices the
		# mask threshold is the only thing turning masking off.
		mask = built.get('in_mask')
		if mask is not None and hasattr(mask.par, 'active'):
			mask.par.active.expr = 'parent().par.Useplayer'

		self.buildPopChain(device, built, spec)
		self.applyMaskAvailability(device, spec)
		return built

	def WireMerge(self):
		"""
		Point World's merge at one device each.

		The merge gathers through a list of POP parameters rather than wires,
		which is what lets the number of devices vary. Paths are read from the
		owner's parent, so `../Device1/null_pointCloud` finds World's sibling.
		"""
		me = self.ownerComp
		merge = me.op('World/mergePOP')
		if merge is None:
			return None
		count = max(1, int(me.par.Numberofdevices))
		# Cook around numBlocks: a sequence block's parameters do not exist
		# until the operator has cooked with the new block count.
		merge.seq.input.numBlocks = count
		merge.cook(force=True)
		for index in range(count):
			merge.par[f'input{index}pop'] = f'../{deviceName(index + 1)}/null_pointCloud'
		return merge

	def RebuildDevices(self):
		"""
		Rebuild the template's sources and chain, then copy every clone again.

		The clones cannot be left to sync themselves. Rebuilding DESTROYS the
		template's operators and makes new ones, and a clone then remakes its
		own copies at their defaults, which quietly keeps every pixel, valid or
		not. So anything that rebuilds the template goes through here.
		"""
		self.destroyDevices()
		self.BuildDeviceSources()
		self.createDevices()
		self.WireMerge()

	def GatherDevices(self, warn=True):
		"""
		Rebuild the device COMPs to match what is plugged in.

		warn=False prints the failure instead of opening a dialog, for anything
		driven from a script: a message box freezes TouchDesigner until it is
		clicked.

		A failed gather CLEARS the devices rather than leaving the old set
		standing. Gather devices means "rebuild the set", so stopping halfway
		would save a rig that sees nothing, and whoever opens the file next
		finds devices that look real and are not. Better to land back on the
		Device1 template alone, with no devices.

		Calibrations are deliberately kept, on success and on failure alike, so
		a camera that drops out and comes back does not cost real work.
		"""
		problem = self.recognizeDevices()
		if problem:
			self.destroyDevices()
			me = self.ownerComp
			me.par.Numberofdevices = 0
			me.par.Devices = ''
			self.SetIds()
			self.WireMerge()
			print(f'[TDXMerger] gather failed: {problem}')
			if warn:
				ui.messageBox('Warning', problem)
			return False
		self.RebuildDevices()
		self.SetIds()
		return True

	def SetIds(self):
		"""Readout of which two cameras the current pair points at."""
		me = self.ownerComp
		devices = [d for d in me.par.Devices.eval().split(',') if d]
		target, source = self.GetPair()
		def label(i):
			return devices[i - 1] if 1 <= i <= len(devices) else '?'
		me.par.Ids = (f'target {target} = {label(target)}   '
			f'source {source} = {label(source)}')

	def GetPair(self):
		"""(target, source). The target is the one that does not move."""
		me = self.ownerComp
		return (int(me.par.Specifypair1), int(me.par.Specifypair2))
