"""
	Device discovery and clone management for TDXDepthCameraMerger.

	Knows nothing about Open3D. Camera specifics live in the deviceTypes table
	so a new camera is a new row, not new code.
"""

DEVICE_PREFIX = 'Device'

# Role named source TOPs, rebuilt per camera type by BuildDeviceSources.
SOURCE_NAMES = ('in_pointcloud', 'in_color', 'in_mask')

# The 0.0.3 hardcoded Kinect selects. Destroyed on the first rebuild.
LEGACY_SOURCE_NAMES = ('kinectazureselect1', 'kinectazureselect2', 'kinectazureselect3')

# Where each source sits, matching the layout from before the abstraction.
SOURCE_POSITIONS = {
	'in_color': (-275, -275),
	'in_mask': (-275, -125),
	'in_pointcloud': (-50, 25),
}


def cameraTopExpr(shortcut, cameraOpName):
	"""
	The `top` expression for a camera select: the nth camera TOP inside whatever
	the Inputsop parameter points at.

	Same shape the component has always used. The only difference is that the
	operator name comes from the deviceTypes row rather than being literal.
	"""
	return ("op(parent.{0}.par.Inputsop).op(f'{1}{{parent().digits}}')"
		" if parent.{0}.par.Inputsop != None else ''").format(shortcut, cameraOpName)


def customTopExpr(shortcut, column):
	"""The `top` expression for a custom device: one customSources row per device."""
	return "parent.{}.CustomSource(parent().digits, '{}')".format(shortcut, column)


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

		A camera with no mask image gets no mask source at all: the Orbbec TOP
		has no player index, its `image` menu is only color/depth/ir/pointcloud.
		`custom` always gets one, because there a mask is a per device cell
		rather than a property of the type.
		"""
		specs = [
			('in_pointcloud', spec['image_pointcloud'], 'pointcloud'),
			('in_color', spec['image_color'], 'color'),
		]
		if spec['type'] == 'custom' or spec['image_mask']:
			specs.append(('in_mask', spec['image_mask'], 'mask'))
		return specs

	def maskIndexExpr(self, spec):
		"""
		What drives switch1, or None to pin it at the raw cloud.

		Index 0 is the raw cloud and 1 the masked one. With no mask source the
		masked branch must never be selected, or multiply1 would blank the whole
		cloud against an unresolved input. For `custom` the answer differs per
		device, so it has to stay an expression rather than a constant.
		"""
		if spec['type'] == 'custom':
			return ("int(parent().par.Useplayer) "
				"if parent().op('in_mask').par.top.eval() else 0")
		if spec['image_mask']:
			return 'parent().par.Useplayer'
		return None

	def wireDeviceSources(self, device, built):
		"""
		Rewire the fixed part of the device chain onto freshly built sources.

		switch1 takes the raw cloud on 0 and the masked one on 1; multiply1 takes
		the cloud on 0 and the thresholded mask on 1. Everything downstream reads
		null_color / null_sourcePointcloud / null_pointCloud, so nothing outside
		this COMP has to change.
		"""
		nodes = {name: device.op(name)
			for name in ('null_color', 'thresh1', 'multiply1', 'switch1')}
		missing = sorted(name for name, o in nodes.items() if o is None)
		if missing:
			raise ValueError('{} is missing {}'.format(device.path, ', '.join(missing)))

		cloud = built['in_pointcloud']
		nodes['null_color'].inputConnectors[0].connect(built['in_color'])
		nodes['switch1'].inputConnectors[0].connect(cloud)
		nodes['multiply1'].inputConnectors[0].connect(cloud)
		nodes['switch1'].inputConnectors[1].connect(nodes['multiply1'])

		mask = built.get('in_mask')
		if mask is None:
			nodes['thresh1'].inputConnectors[0].disconnect()
			nodes['multiply1'].inputConnectors[1].disconnect()
		else:
			nodes['thresh1'].inputConnectors[0].connect(mask)
			nodes['multiply1'].inputConnectors[1].connect(nodes['thresh1'])

	def applyMaskAvailability(self, device, spec):
		"""
		Point switch1 at the right branch, and tell the UI whether masking is
		possible at all for this camera.
		"""
		index = device.op('switch1').par.index
		expr = self.maskIndexExpr(spec)
		if expr is None:
			index.mode = ParMode.CONSTANT
			index.val = 0
		else:
			index.expr = expr

		me = self.ownerComp
		if spec['type'] == 'custom':
			table = me.op('customSources')
			available = table is not None and any(table[r, 'mask'].val.strip()
				for r in range(1, table.numRows))
		else:
			available = bool(spec['image_mask'])
		me.par.Usemaskforcalibration.enable = available

	def recognizeDevices(self):
		"""
		Ask the camera TOP which physical devices exist. Which parameter holds
		that list differs per camera (kinectazure uses `sensor`, orbbec uses
		`device`), so it comes from the deviceTypes table.
		"""
		me = self.ownerComp
		spec = self.deviceTypeRow()

		if spec['type'] == 'custom':
			table = me.op('customSources')
			count = max(0, table.numRows - 1) if table else 0
			if not count:
				ui.messageBox('Warning', 'Device type is "custom" but the '
					'customSources table is empty. Add one row per device.')
				return False
			me.par.Numberofdevices = count
			me.par.Devices = ','.join(
				table[r, 0].val for r in range(1, table.numRows))
			return True

		target = me.op(me.par.Inputsop)
		if target is None:
			ui.messageBox('Warning', 'Inputs op does not point at anything. Set it to '
				'the base holding your camera TOPs.')
			return False

		first = target.op('{}1'.format(spec['cameraop_name']))
		if first is None:
			ui.messageBox('Warning', 'Could not find a "{}1" TOP inside {}.'.format(
				spec['cameraop_name'], target.path))
			return False

		parameter = getattr(first.par, spec['devicepar'], None)
		if parameter is None:
			ui.messageBox('Warning', '{} has no "{}" parameter.'.format(
				first.path, spec['devicepar']))
			return False

		names = [n for n in (parameter.menuNames or []) if n]
		if not names:
			ui.messageBox('Warning', 'No cameras found. Is anything plugged in?')
			return False

		me.par.Numberofdevices = len(names)
		me.par.Devices = ','.join(names)
		return True

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
		template = me.op('{}1'.format(DEVICE_PREFIX))
		if template is None:
			raise ValueError('no {}1 template in {}'.format(DEVICE_PREFIX, me.path))
		for index in range(2, int(me.par.Numberofdevices) + 1):
			clone = me.copy(template, name='{}{}'.format(DEVICE_PREFIX, index))
			clone.nodeX = template.nodeX
			clone.nodeY = template.nodeY - 200 * (index - 1)
			clone.par.clone = template.name

	# ____ Public ____

	def CustomSource(self, index, column):
		"""
		Resolve one customSources cell to an operator, or None.

		Called from every source TOP of a `custom` device, so one table row is
		one device and the row index is the device's trailing digit.
		"""
		table = self.ownerComp.op('customSources')
		if table is None or not 0 < index < table.numRows:
			return None
		path = table[index, column].val.strip()
		return self.ownerComp.op(path) if path else None

	def BuildDeviceSources(self, device=None):
		"""
		Rebuild a device's source TOPs from the current deviceTypes row.

		This is what makes a new camera a table row rather than new code. Only
		the Device1 template needs it: clones replicate the master's children,
		and each source resolves its own camera through `parent().digits`.
		"""
		me = self.ownerComp
		if device is None:
			device = me.op('{}1'.format(DEVICE_PREFIX))
		if device is None:
			raise ValueError('no {}1 template in {}'.format(DEVICE_PREFIX, me.path))

		spec = self.deviceTypeRow()
		shortcut = me.par.parentshortcut.eval()
		custom = spec['type'] == 'custom'

		for name in SOURCE_NAMES + LEGACY_SOURCE_NAMES:
			stale = device.op(name)
			if stale is not None:
				stale.destroy()

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

		# Colour aligned to depth. Only the Kinect select can do it, the Orbbec
		# select has no such parameter, so this is a capability test not a flag.
		if hasattr(built['in_color'].par, 'remapimage'):
			built['in_color'].par.remapimage = True

		# A plain selectTOP has no `active`, so for custom devices the switch1
		# expression is the only thing gating the masked branch.
		mask = built.get('in_mask')
		if mask is not None and hasattr(mask.par, 'active'):
			mask.par.active.expr = 'parent().par.Useplayer'

		self.wireDeviceSources(device, built)
		self.applyMaskAvailability(device, spec)
		return built

	def GatherDevices(self):
		"""Rebuild the device COMPs to match what is plugged in."""
		if not self.recognizeDevices():
			return False
		self.destroyDevices()
		self.BuildDeviceSources()
		self.createDevices()
		self.SetIds()
		return True

	def SetIds(self):
		"""Readout of which two cameras the current pair refers to."""
		me = self.ownerComp
		devices = [d for d in me.par.Devices.eval().split(',') if d]
		target, source = self.GetPair()
		def label(i):
			return devices[i - 1] if 1 <= i <= len(devices) else '?'
		me.par.Ids = 'target {} = {}   source {} = {}'.format(
			target, label(target), source, label(source))

	def GetPair(self):
		"""(target, source). The target is the reference and does not move."""
		me = self.ownerComp
		return (int(me.par.Specifypair1), int(me.par.Specifypair2))
