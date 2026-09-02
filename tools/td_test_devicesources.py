"""
Checks the camera handling INSIDE TouchDesigner, with no camera attached.

Part A sets Devicetype to every row of the deviceTypes table and checks what
BuildDeviceSources made: operator types, image settings, which attribute each
input writes, the mask threshold, and how the chain is wired. It calls the
builder directly, so everything can be checked in the same frame.

A parameter callback does NOT run inside the script that changed the parameter,
it runs at the end of the frame. So the rebuild trigger cannot be checked
inline. Part C schedules that check and leaves its verdict on the component;
read it a few frames later with

    op('/ProjectName/TDXDepthCamMerger').fetch('triggerResults')

Part B is the whole thing end to end: point customSources at real TOPs and run
a real calibration through the component. It needs the made up scene from
td_test_calibrate.py, so run that first:

    exec(open(r'.../td_test_calibrate.py', encoding='utf-8').read())
    exec(open(r'.../td_test_devicesources.py', encoding='utf-8').read())

Puts the component back on kinectazure with an empty customSources on the way
out.
"""

import numpy as np

COMP = op('/ProjectName/TDXDepthCamMerger')
RIG = op('/ProjectName/calibtest')

# What a customSources row points at here. It has to be a TOP: points are made
# inside the device, so the component's inputs are images for every type.
CLOUD = 'cloud_source'

# The point chain, in order, and what feeds what.
POP_CHAIN = (
	('pop_convert', 'toptoPOP', None),
	('pop_valid', 'deletePOP', 'pop_convert'),
	('null_sourcePointcloud', 'nullPOP', 'pop_valid'),
	('pop_transform', 'transformPOP', 'null_sourcePointcloud'),
	('pop_show', 'deletePOP', 'pop_transform'),
	('null_pointCloud', 'nullPOP', 'pop_show'),
)

MASK_THRESHOLD = 0.036
MASK_OFF = 1e9

# Radius of the origin sphere pop_valid throws away, in metres. Tracks
# ORIGIN_RADIUS in extUtilities.
ORIGIN_RADIUS = 0.001

results = []


def check(name, ok, detail=''):
	results.append((name, bool(ok), detail))
	print('  [{}] {}{}'.format('PASS' if ok else 'FAIL', name,
		('  -> ' + detail) if detail else ''))


def inputName(o, index):
	"""Name of whatever is wired into input `index`, or None."""
	con = o.inputConnectors[index]
	return con.connections[0].owner.name if con.connections else None


def convertBlocks(convert):
	"""What each of the conversion's inputs reads: (source, channels, attribute)."""
	return [(convert.par['input{}top'.format(i)].eval().name
			if convert.par['input{}top'.format(i)].eval() else None,
		convert.par['input{}chanscope'.format(i)].eval(),
		convert.par['input{}attrscope'.format(i)].eval())
		for i in range(convert.seq.input.numBlocks)]


def rowSpec(kind):
	table = COMP.op('deviceTypes')
	row = table.row(kind)
	return {table[0, c].val: row[c].val for c in range(table.numCols)}


def poseError(a, b):
	d = np.linalg.inv(a) @ b
	return (np.linalg.norm(d[:3, 3]),
		np.degrees(np.arccos(np.clip((np.trace(d[:3, :3]) - 1) / 2, -1, 1))))


def readMatrix(index):
	"""The 4x4 a device ended up with."""
	table = COMP.op('Device{}/transformMatrix'.format(index))
	return np.array([[float(table[r, c].val) for c in range(4)] for r in range(4)])


def cookDevice(index):
	"""Cook one device's chain, which nothing else does here."""
	device = COMP.op('Device{}'.format(index))
	for name, _, _ in POP_CHAIN:
		if device.op(name):
			device.op(name).cook(force=True)
	return device


# ____________________________________________________________ preconditions

print('\npreconditions')
check('component found', COMP is not None)
check('extension bound', hasattr(COMP, 'BuildDeviceSources'),
	'BuildDeviceSources is promoted from extUtilities')
check('RebuildDevices promoted', hasattr(COMP, 'RebuildDevices'),
	'the only safe way to retype the template once clones exist')
device = COMP.op('Device1')
check('Device1 present', device is not None)


# ____________________________________________________________ A. per type

for kind in ('kinectazure', 'orbbec', 'zed', 'custom'):
	print('\nA. Devicetype = {}'.format(kind))
	spec = rowSpec(kind)
	wantsMask = kind == 'custom' or bool(spec['image_mask'])

	# Called directly: the parameter callback would only run next frame.
	COMP.par.Devicetype = kind
	COMP.RebuildDevices()
	device = COMP.op('Device1')

	names = [o.name for o in device.children if o.name.startswith('in_')]
	check('{}: no legacy kinect selects left'.format(kind),
		not [o for o in device.children if o.name.startswith('kinectazureselect')])
	check('{}: no TOP era chain left'.format(kind),
		not [n for n in ('null_color', 'thresh1', 'multiply1', 'switch1', 'glsl2')
			if device.op(n)],
		'the mask branch and the transform shader are gone')
	check('{}: pointcloud and colour sources exist'.format(kind),
		{'in_pointcloud', 'in_color'}.issubset(set(names)), str(sorted(names)))
	check('{}: mask source {}'.format(kind, 'present' if wantsMask else 'absent'),
		('in_mask' in names) == wantsMask, str(sorted(names)))

	for name in names:
		check('{}: {} is a {}'.format(kind, name, spec['selectop']),
			device.op(name).OPType == spec['selectop'], device.op(name).OPType)
		# Which parameter names the camera TOP is per camera: ZED's select calls
		# it zedtop, everyone else's calls it top. Writing the wrong one raises
		# rather than leaving a source pointing at nothing.
		sourcePar = spec.get('sourcepar') or 'top'
		par = getattr(device.op(name).par, sourcePar, None)
		check('{}: {} reads its source through "{}"'.format(kind, name, sourcePar),
			par is not None and par.mode == ParMode.EXPRESSION and bool(par.expr),
			repr(par.expr)[:60] if par is not None else 'no such parameter')

	if kind != 'custom':
		check('{}: pointcloud image = {}'.format(kind, spec['image_pointcloud']),
			device.op('in_pointcloud').par.image.eval() == spec['image_pointcloud'])
		check('{}: colour image = {}'.format(kind, spec['image_color']),
			device.op('in_color').par.image.eval() == spec['image_color'])
		if wantsMask:
			check('{}: mask image = {}'.format(kind, spec['image_mask']),
				device.op('in_mask').par.image.eval() == spec['image_mask'])

	# Only the Kinect select can line colour up with depth.
	hasRemap = hasattr(device.op('in_color').par, 'remapimage')
	check('{}: colour aligned to depth {}'.format(
		kind, 'on' if hasRemap else 'not offered by this TOP'),
		device.op('in_color').par.remapimage.eval() if hasRemap else True)

	# The point chain: types, order and wiring.
	for name, optype, feeder in POP_CHAIN:
		node = device.op(name)
		check('{}: {} is a {}'.format(kind, name, optype),
			node is not None and node.OPType == optype,
			node.OPType if node else 'missing')
		if feeder:
			check('{}: {} <- {}'.format(kind, name, feeder),
				inputName(node, 0) == feeder, str(inputName(node, 0)))

	convert = device.op('pop_convert')
	blocks = convertBlocks(convert)
	check('{}: P comes from the point cloud RGB'.format(kind),
		blocks[0] == ('in_pointcloud', 'r g b', 'P'), str(blocks[0]))
	check('{}: Color comes from the colour RGBA'.format(kind),
		blocks[1] == ('in_color', 'r g b a', 'Color'),
		'rgba, not rgb: Color is a float4 and three channels leaves a warning')
	check('{}: valid comes from the point cloud alpha'.format(kind),
		blocks[2] == ('in_pointcloud', 'a', 'valid.x'),
		'the component scope is load bearing, a bare "valid" writes zero')
	check('{}: {} input blocks'.format(kind, 4 if wantsMask else 3),
		len(blocks) == (4 if wantsMask else 3), str(len(blocks)))
	if wantsMask:
		check('{}: maskv comes from the mask alpha'.format(kind),
			blocks[3] == ('in_mask', 'a', 'maskv.x'), str(blocks[3]))
	check('{}: pop_convert is clean'.format(kind),
		not convert.errors().strip() and not convert.warnings().strip(),
		(convert.errors() + convert.warnings()).strip()[:80])

	keep = device.op('pop_valid')
	check('{}: pop_valid keeps rather than deletes'.format(kind),
		keep.par.invert.eval() == 'keep' and keep.par.entity.eval() == 'point')
	check('{}: pop_valid drops the invalid pixels'.format(kind),
		(keep.par.attr0inattr.eval(), keep.par.attr0func.eval(),
			keep.par.attr0value.eval()) == ('valid.x', 'gte', 0.5))
	# Second, independent reason to drop a point: an unreturned pixel sits at
	# exactly (0, 0, 0), and this runs before pop_transform so the origin is
	# the sensor. Alpha is undocumented for both cameras TD ships, geometry is
	# not.
	check('{}: pop_valid also drops points at the camera origin'.format(kind),
		keep.seq.bound.numBlocks == 1
		and keep.par.bound0enabled.eval()
		and keep.par.bound0inattr.eval() == 'P'
		and keep.par.bound0type.eval() == 'boundingsphere'
		and keep.par.bound0invert.eval()
		and abs(keep.par.bound0scalex.eval() - ORIGIN_RADIUS) < 1e-9
		and abs(keep.par.bound0translatex.eval()) < 1e-9,
		'{} block(s), r={}'.format(keep.seq.bound.numBlocks,
			keep.par.bound0scalex.eval()))
	if not wantsMask:
		# Orbbec publishes no player index, and ZED's body mask is deliberately
		# not wired, so both come out with the invalid-pixel condition alone.
		check('{}: no mask condition at all'.format(kind),
			keep.seq.attr.numBlocks == 1,
			'{} blocks; this camera type supplies no mask'.format(keep.seq.attr.numBlocks))
	else:
		check('{}: mask condition present and expression driven'.format(kind),
			keep.seq.attr.numBlocks == 2
			and keep.par.attr1value.mode == ParMode.EXPRESSION
			and 'Useplayer' in keep.par.attr1value.expr,
			repr(keep.par.attr1value.expr)[:70])
		check('{}: the mask condition reads BELOW the threshold'.format(kind),
			(keep.par.attr1inattr.eval(), keep.par.attr1func.eval(),
				keep.par.attr1combine.eval()) == ('maskv.x', 'lt', 'and'),
			'a Kinect body index is small, the background is 255')

	show = device.op('pop_show')
	check('{}: Show gates the chain AFTER the sample point'.format(kind),
		show.par.attr0value.mode == ParMode.EXPRESSION
		and 'Show' in show.par.attr0value.expr,
		'a hidden device must still be calibratable')

	check('{}: pop_transform reads transformMatrix'.format(kind),
		device.op('pop_transform').par.xformmatrixop.eval() is device.op('transformMatrix'),
		str(device.op('pop_transform').par.xformmatrixop.eval()))

	check('{}: the POP chain is clone immune'.format(kind),
		all(device.op(n).cloneImmune for n, _, _ in POP_CHAIN),
		'a clone sync resets sequence blocks to the factory defaults')

	check('{}: Usemaskforcalibration enable = {}'.format(kind, bool(spec['image_mask'])),
		COMP.par.Usemaskforcalibration.enable == bool(spec['image_mask']),
		'no mask source means the toggle is greyed out')

	# Every device type reads Inputs op, custom included: it is the base the
	# customSources cells are looked up in.
	check('{}: Inputsop stays enabled'.format(kind), COMP.par.Inputsop.enable is True)


# ______________________________________ A2. custom, mask on one device only

# The trickiest part of the design. Clones copy the template, so two custom
# devices cannot hold different operators. A difference between them has to come
# from the mask threshold expression, and this is what proves it does.

print('\nA2. custom with a mask on device 1 only')
table = COMP.op('customSources')
if RIG is None:
	check('calibtest rig present', False, 'run td_test_calibrate.py first, skipping A2')
else:
	table.clear(keepFirstRow=True)
	for i in (1, 2):
		table.appendRow(['dev{}'.format(i),
			'{}/Device{}/{}'.format(RIG.path, i, CLOUD), '', ''])
	table[1, 'mask'] = '{}/Device1/{}'.format(RIG.path, CLOUD)

	COMP.par.Devicetype = 'custom'
	COMP.par.Usemaskforcalibration = True
	COMP.GatherDevices()

	check('custom: sources really are plain selects',
		COMP.op('Device1/in_pointcloud').OPType == 'selectTOP',
		COMP.op('Device1/in_pointcloud').OPType)
	check('custom: two devices built from two rows',
		COMP.op('Device2') is not None and int(COMP.par.Numberofdevices) == 2)
	check('custom: one masked row re-enables the toggle',
		COMP.par.Usemaskforcalibration.enable is True)
	check('custom: both devices hold the same operators',
		COMP.op('Device2/in_mask') is not None,
		'clones cannot differ in structure, only in what the expressions resolve to')
	check('custom: the masked device applies the threshold',
		abs(COMP.op('Device1/pop_valid').par.attr1value.eval() - MASK_THRESHOLD) < 1e-9,
		'row 1 supplies a mask TOP')
	check('custom: the unmasked device switches the condition off',
		COMP.op('Device2/pop_valid').par.attr1value.eval() == MASK_OFF,
		'row 2 has an empty mask cell, so the threshold is out of reach')
	check('custom: an empty mask cell still feeds pop_convert',
		COMP.op('Device2/in_mask').par.top.eval() is COMP.op('Device2/in_pointcloud').par.top.eval(),
		'it falls back to the cloud, so no input block is left unresolved')
	check('custom: neither device warns',
		not COMP.op('Device1/pop_convert').warnings().strip()
		and not COMP.op('Device2/pop_convert').warnings().strip(),
		(COMP.op('Device2/pop_convert').warnings()).strip()[:80])
	COMP.par.Usemaskforcalibration = False

	# A2b. customSources cells are looked up inside Inputs op, so a whole set of
	# test TOPs is swapped by repointing that one parameter. Full paths have to
	# keep working too.
	keepInputs = COMP.par.Inputsop.eval()
	absolute = COMP.op('Device1/in_pointcloud').par.top.evalOPs()
	check('custom: an absolute path resolves with Inputs op empty',
		bool(absolute) and absolute[0].path == '{}/Device1/{}'.format(RIG.path, CLOUD),
		absolute[0].path if absolute else 'nothing')

	table.clear(keepFirstRow=True)
	for i in (1, 2):
		table.appendRow(['dev{}'.format(i), CLOUD, '', ''])
	COMP.par.Inputsop = RIG.op('Device1').path
	COMP.RebuildDevices()
	viaBase = COMP.op('Device1/in_pointcloud').par.top.evalOPs()
	check('custom: a plain name resolves inside Inputs op',
		bool(viaBase) and viaBase[0].path == '{}/Device1/{}'.format(RIG.path, CLOUD),
		viaBase[0].path if viaBase else 'nothing')

	COMP.par.Inputsop = RIG.op('Device2').path
	COMP.RebuildDevices()
	swapped = COMP.op('Device1/in_pointcloud').par.top.evalOPs()
	check('custom: repointing Inputs op swaps the whole set',
		bool(swapped) and swapped[0].path == '{}/Device2/{}'.format(RIG.path, CLOUD),
		swapped[0].path if swapped else 'nothing')

	# A2b2. Rebuilding the template destroys its operators, and a clone then
	# remakes its own copies at their defaults, quietly losing the inputs set up
	# above and keeping every pixel. RebuildDevices copies the clones again,
	# which is the only reason the clone below still has four inputs.
	clone = COMP.op('Device2/pop_convert')
	check('custom: a clone survives a template rebuild with its sequences',
		clone.seq.input.numBlocks == 4 and clone.seq.attr.numBlocks == 3,
		'{} input / {} attribute blocks'.format(
			clone.seq.input.numBlocks, clone.seq.attr.numBlocks))

	# A row that points at nothing has to be refused, not turned into a device
	# that sees nothing. Asked through the helper rather than GatherDevices,
	# whose dialog would block a scripted run.
	util = COMP.ext.extUtilities
	COMP.par.Inputsop = ''
	dead = util.UnresolvedCustomRows()
	check('custom: a plain name with no Inputs op is reported dead',
		len(dead) == 2, '{} of 2 rows'.format(len(dead)))

	COMP.par.Inputsop = RIG.op('Device1').path
	check('custom: the same rows are live once Inputs op is set',
		util.UnresolvedCustomRows() == [], str(util.UnresolvedCustomRows()))

	COMP.par.Inputsop = keepInputs
	table.clear(keepFirstRow=True)
	for i in (1, 2):
		table.appendRow(['dev{}'.format(i),
			'{}/Device{}/{}'.format(RIG.path, i, CLOUD), '', ''])
	check('custom: an absolute path needs no Inputs op',
		util.UnresolvedCustomRows() == [], str(util.UnresolvedCustomRows()))
	COMP.RebuildDevices()

	# A2c. A failed gather has to leave a clean state: keeping the old devices
	# would save a rig that sees nothing. warn=False keeps the dialog out of a
	# scripted run.
	COMP.par.Inputsop = ''
	table.clear(keepFirstRow=True)
	table.appendRow(['dev1', 'no_such_top', '', ''])
	calRows = COMP.op('calibrationData').numRows
	check('custom: a gather with nothing resolvable returns False',
		COMP.GatherDevices(warn=False) is False)
	check('custom: a failed gather clears the devices',
		COMP.op('Device2') is None and int(COMP.par.Numberofdevices) == 0,
		'Numberofdevices {}, Device2 {}'.format(
			COMP.par.Numberofdevices.eval(), COMP.op('Device2')))
	check('custom: a failed gather keeps the Device1 template',
		COMP.op('Device1') is not None)
	check('custom: a failed gather does NOT throw the calibration away',
		COMP.op('calibrationData').numRows == calRows,
		'{} rows before, {} after'.format(calRows, COMP.op('calibrationData').numRows))
	check('custom: a failed gather leaves the merge on one block',
		COMP.op('World/mergePOP').seq.input.numBlocks == 1,
		str(COMP.op('World/mergePOP').seq.input.numBlocks))

	COMP.par.Inputsop = keepInputs
	table.clear(keepFirstRow=True)
	for i in (1, 2):
		table.appendRow(['dev{}'.format(i),
			'{}/Device{}/{}'.format(RIG.path, i, CLOUD), '', ''])
	check('custom: gathering recovers after the failure',
		COMP.GatherDevices(warn=False) is True and int(COMP.par.Numberofdevices) == 2,
		'Numberofdevices {}'.format(COMP.par.Numberofdevices.eval()))


# ____________________________________________________________ B. end to end

print('\nB. custom end to end through the real component')
if RIG is None:
	check('calibtest rig present', False, 'run td_test_calibrate.py first, skipping B')
else:
	TRUE12 = RIG.fetch('TRUE12', None)
	check('ground truth available on the rig', TRUE12 is not None)

	table.clear(keepFirstRow=True)
	for i in (1, 2, 3):
		table.appendRow(['dev{}'.format(i),
			'{}/Device{}/{}'.format(RIG.path, i, CLOUD), '', ''])

	COMP.par.Devicetype = 'custom'
	COMP.par.Usecoloricp = False
	COMP.par.Referencedevice = 1
	COMP.par.Specifypair1, COMP.par.Specifypair2 = 1, 2
	gathered = COMP.GatherDevices()
	check('GatherDevices succeeded for custom', gathered is True)
	check('found 3 devices from customSources', int(COMP.par.Numberofdevices) == 3,
		str(COMP.par.Numberofdevices.eval()))

	counts = []
	for i in (1, 2, 3):
		dev = cookDevice(i)
		pop = dev.op('null_sourcePointcloud')
		counts.append(pop.numPoints() if pop else 0)
		check('Device{} resolves a cloud through the custom select'.format(i),
			bool(counts[-1]), '{} points'.format(counts[-1]))

	# What reaches the worker is points, not pixels. The clouds here are padded
	# out to a 256x192 image, so anything that stopped deleting would report the
	# full 49152.
	check('the sampled clouds carry real points, not padded texels',
		all(0 < n < 256 * 192 for n in counts), str(counts))

	# The origin sphere is a second, independent guard. Widen the alpha
	# condition until it keeps every texel: the count must not move, because
	# the pixels the camera never returned sit at exactly (0, 0, 0). This is
	# what a camera whose point cloud alpha is not validity would look like.
	keep = cookDevice(1).op('pop_valid')
	byAlpha = keep.numPoints()
	keep.par.attr0value = -1.0
	keep.cook(force=True)
	byGeometry = keep.numPoints()
	keep.par.attr0value = 0.5
	keep.cook(force=True)
	check('the origin sphere drops the invalid pixels on its own',
		byGeometry == byAlpha and keep.numPoints() == byAlpha,
		'{} points by alpha, {} with the alpha test defeated'.format(
			byAlpha, byGeometry))

	merge = COMP.op('World/mergePOP')
	merged = COMP.op('World/merged')
	for o in (merge, merged, COMP.op('out_pop')):
		o.cook(force=True)
	check('mergePOP gathers one block per device',
		merge.seq.input.numBlocks == 3, str(merge.seq.input.numBlocks))
	check('the merged cloud is every device summed',
		merged.numPoints() == sum(
			cookDevice(i).op('null_pointCloud').numPoints() for i in (1, 2, 3)),
		'{} points'.format(merged.numPoints()))
	check('the merged cloud carries P and Color only',
		sorted(a.name for a in merged.pointAttributes) == ['Color', 'P'],
		str(sorted(a.name for a in merged.pointAttributes)))
	check('out_pop is the component output',
		COMP.op('out_pop').numPoints() == merged.numPoints(),
		'{} points'.format(COMP.op('out_pop').numPoints()))

	COMP.Calibrate(pair=(1, 2), mode='globalRegistration')
	fit = float(COMP.par.Lastfitness.eval() or 0)
	glob = readMatrix(2)
	gt, gr = poseError(TRUE12, glob)
	check('global registration fitness > 0.85', fit > 0.85, 'fitness {:.3f}'.format(fit))
	check('global registration within 0.15 m / 4 deg', gt < 0.15 and gr < 4.0,
		'{:.4f} m, {:.3f} deg'.format(gt, gr))

	COMP.Refine(pair=(1, 2))
	fit = float(COMP.par.Lastfitness.eval() or 0)
	icp = readMatrix(2)
	it, ir = poseError(TRUE12, icp)
	check('ICP fitness > 0.9', fit > 0.9, 'fitness {:.3f}'.format(fit))
	check('ICP within 0.01 m / 0.1 deg', it < 0.01 and ir < 0.1,
		'{:.6f} m, {:.4f} deg'.format(it, ir))
	check('ICP improved on the global result', it <= gt and ir <= gr,
		'{:.4f} -> {:.6f} m,  {:.3f} -> {:.4f} deg'.format(gt, it, gr, ir))

	# Show hides a device from the merged cloud but must not stop it being
	# calibrated: the cloud is sampled before it.
	COMP.op('Device2').par.Show = False
	visible = cookDevice(2).op('null_pointCloud').numPoints()
	sampled = COMP.op('Device2/null_sourcePointcloud').numPoints()
	check('Show off empties the device downstream', visible == 0, str(visible))
	check('Show off leaves the calibration sample intact', sampled == counts[1],
		'{} points still sampled'.format(sampled))
	COMP.op('Device2').par.Show = True

	# One pulse doing both stages in a single worker run. It has to land where
	# two separate pulses land, and report both stages.
	COMP.ResetCalibration()
	COMP.Calibrate(pair=(1, 2), mode='globalThenIcp')
	chained = readMatrix(2)
	ct, cr = poseError(TRUE12, chained)
	cfit = float(COMP.par.Lastfitness.eval() or 0)
	check('chained mode reaches ICP accuracy in one pulse', ct < 0.01 and cr < 0.1,
		'{:.6f} m, {:.4f} deg'.format(ct, cr))
	check('chained mode fitness > 0.9', cfit > 0.9, 'fitness {:.3f}'.format(cfit))
	row = COMP.op('calibrationData').row('2')
	check('chained mode is recorded as its own method',
		row is not None and COMP.op('calibrationData')[row[0].row, 'method'].val
		== 'globalThenIcp',
		COMP.op('calibrationData')[row[0].row, 'method'].val if row else 'no row')
	check('status line reports both stages',
		'global stage' in COMP.par.Laststatus.eval(),
		COMP.par.Laststatus.eval()[:90])


# ____________________________________________________________ restore

print('\nrestoring the component')
table.clear(keepFirstRow=True)
COMP.par.Devicetype = 'kinectazure'
COMP.par.Numberofdevices = 1
COMP.par.Devices = ''
COMP.par.Usemaskforcalibration = False
COMP.RebuildDevices()
COMP.ResetCalibration()
check('back on kinectazure with kinect selects',
	COMP.op('Device1/in_pointcloud').OPType == 'kinectazureselectTOP')
check('customSources emptied', COMP.op('customSources').numRows == 1)
check('device clones cleared', COMP.op('Device2') is None)
check('merge back on one block', COMP.op('World/mergePOP').seq.input.numBlocks == 1)

failed = [n for n, ok, _ in results if not ok]
print('\n{} checks, {} failed'.format(len(results), len(failed)))
for n in failed:
	print('  FAIL {}'.format(n))


# ____________________________________________ C. the parexec1 rebuild trigger

# Changing Devicetype on its own has to rebuild the template. That happens in a
# parameter callback, which runs at the end of the frame rather than inside this
# script, so the check has to wait a few frames. The wait also lets the
# callbacks queued by everything above finish.

COMP.store('triggerResults', [])

STEP = ("c = op('/ProjectName/TDXDepthCamMerger')\n"
	"c.store('triggerResults', c.fetch('triggerResults', []) + [({!r}, bool({}))])")


def schedule(code, frames):
	run(code, delayFrames=frames, fromOP=COMP)


schedule("op('/ProjectName/TDXDepthCamMerger').par.Devicetype = 'orbbec'", 10)
schedule(STEP.format('Devicetype -> orbbec rebuilds via parexec1',
	"c.op('Device1/in_pointcloud').OPType == 'orbbecselectTOP' "
	"and c.op('Device1/in_mask') is None "
	"and c.op('Device1/pop_convert').seq.input.numBlocks == 3 "
	"and c.op('Device1/pop_valid').seq.attr.numBlocks == 1"), 14)
schedule("op('/ProjectName/TDXDepthCamMerger').par.Devicetype = 'kinectazure'", 18)
schedule(STEP.format('Devicetype -> kinectazure rebuilds via parexec1',
	"c.op('Device1/in_pointcloud').OPType == 'kinectazureselectTOP' "
	"and c.op('Device1/in_mask') is not None "
	"and c.op('Device1/pop_convert').seq.input.numBlocks == 4 "
	"and c.op('Device1/pop_valid').par.attr1value.mode == ParMode.EXPRESSION"), 22)

print('\nC. parexec1 trigger scheduled. In a few frames, read:')
print("   op('/ProjectName/TDXDepthCamMerger').fetch('triggerResults')")
