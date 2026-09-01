"""
Checks the camera abstraction INSIDE TouchDesigner, with no camera attached.

Part A drives Devicetype through every row of the deviceTypes table and asserts
what BuildDeviceSources actually produced: operator types, image values, the
masked/raw switch, and the full wiring of the fixed part of the device chain.
It calls the builder directly so the checks are deterministic within one frame.

A parameterexecuteDAT callback does NOT run inside the script that changed the
parameter, it runs at the end of the frame. So the parexec1 rebuild trigger
cannot be asserted inline. Part C schedules that check with delayed run() calls
and stores its verdict on the component; read it a few frames later with

    op('/ProjectName/TDXDepthCameraMerger').fetch('triggerResults')

Part B is the headline new capability end to end: point customSources at real
TOPs and run a real global registration plus ICP refine through the component.
It needs the synthetic scene from td_test_calibrate.py, so run that first:

    exec(open(r'.../td_test_calibrate.py', encoding='utf-8').read())
    exec(open(r'.../td_test_devicesources.py', encoding='utf-8').read())

Restores the component to kinectazure with an empty customSources on the way out.
"""

import numpy as np

COMP = op('/ProjectName/TDXDepthCameraMerger')
RIG = op('/ProjectName/calibtest')
CLOUD = 'null_sourcePointcloud'

results = []


def check(name, ok, detail=''):
	results.append((name, bool(ok), detail))
	print('  [{}] {}{}'.format('PASS' if ok else 'FAIL', name,
		('  -> ' + detail) if detail else ''))


def inputName(o, index):
	"""Name of whatever is wired into input `index`, or None."""
	con = o.inputConnectors[index]
	return con.connections[0].owner.name if con.connections else None


def rowSpec(kind):
	table = COMP.op('deviceTypes')
	row = table.row(kind)
	return {table[0, c].val: row[c].val for c in range(table.numCols)}


def poseError(a, b):
	d = np.linalg.inv(a) @ b
	return (np.linalg.norm(d[:3, 3]),
		np.degrees(np.arccos(np.clip((np.trace(d[:3, :3]) - 1) / 2, -1, 1))))


def readMatrix(index):
	"""The composed 4x4 a device ended up with."""
	table = COMP.op('Device{}/transformMatrix'.format(index))
	return np.array([[float(table[r, c].val) for c in range(4)] for r in range(4)])


# ____________________________________________________________ preconditions

print('\npreconditions')
check('component found', COMP is not None)
check('extension bound', hasattr(COMP, 'BuildDeviceSources'),
	'BuildDeviceSources is promoted from extUtilities')
device = COMP.op('Device1')
check('Device1 present', device is not None)


# ____________________________________________________________ A. per type

for kind in ('kinectazure', 'orbbec', 'custom'):
	print('\nA. Devicetype = {}'.format(kind))
	spec = rowSpec(kind)
	wantsMask = kind == 'custom' or bool(spec['image_mask'])

	# Called directly, not left to parexec1, which would only fire next frame.
	COMP.par.Devicetype = kind
	COMP.BuildDeviceSources()

	names = [o.name for o in device.children if o.name.startswith('in_')]
	check('{}: no legacy kinect selects left'.format(kind),
		not [o for o in device.children if o.name.startswith('kinectazureselect')])
	check('{}: pointcloud and colour sources exist'.format(kind),
		{'in_pointcloud', 'in_color'}.issubset(set(names)), str(sorted(names)))
	check('{}: mask source {}'.format(kind, 'present' if wantsMask else 'absent'),
		('in_mask' in names) == wantsMask, str(sorted(names)))

	for name in names:
		check('{}: {} is a {}'.format(kind, name, spec['selectop']),
			device.op(name).OPType == spec['selectop'], device.op(name).OPType)

	if kind != 'custom':
		check('{}: pointcloud image = {}'.format(kind, spec['image_pointcloud']),
			device.op('in_pointcloud').par.image.eval() == spec['image_pointcloud'])
		check('{}: colour image = {}'.format(kind, spec['image_color']),
			device.op('in_color').par.image.eval() == spec['image_color'])
		if wantsMask:
			check('{}: mask image = {}'.format(kind, spec['image_mask']),
				device.op('in_mask').par.image.eval() == spec['image_mask'])

	# remapimage is a Kinect only capability, not a flag in the table.
	hasRemap = hasattr(device.op('in_color').par, 'remapimage')
	check('{}: colour aligned to depth {}'.format(
		kind, 'on' if hasRemap else 'not offered by this TOP'),
		device.op('in_color').par.remapimage.eval() if hasRemap else True)

	# Wiring: switch1 takes raw on 0 and masked on 1.
	check('{}: null_color <- in_color'.format(kind),
		inputName(device.op('null_color'), 0) == 'in_color')
	check('{}: switch1[0] <- in_pointcloud (raw)'.format(kind),
		inputName(device.op('switch1'), 0) == 'in_pointcloud')
	check('{}: multiply1[0] <- in_pointcloud'.format(kind),
		inputName(device.op('multiply1'), 0) == 'in_pointcloud')
	check('{}: switch1[1] <- multiply1 (masked)'.format(kind),
		inputName(device.op('switch1'), 1) == 'multiply1')
	check('{}: thresh1[0] <- {}'.format(kind, 'in_mask' if wantsMask else 'nothing'),
		inputName(device.op('thresh1'), 0) == ('in_mask' if wantsMask else None))
	check('{}: multiply1[1] <- {}'.format(kind, 'thresh1' if wantsMask else 'nothing'),
		inputName(device.op('multiply1'), 1) == ('thresh1' if wantsMask else None))

	index = device.op('switch1').par.index
	if kind == 'orbbec':
		check('orbbec: switch1 index pinned to raw',
			index.mode == ParMode.CONSTANT and index.eval() == 0,
			'{} {}'.format(index.mode, index.eval()))
	else:
		check('{}: switch1 index follows Useplayer'.format(kind),
			index.mode == ParMode.EXPRESSION and 'Useplayer' in index.expr,
			repr(index.expr))

	check('{}: Usemaskforcalibration enable = {}'.format(kind, bool(spec['image_mask'])),
		COMP.par.Usemaskforcalibration.enable == bool(spec['image_mask']),
		'no mask source means the toggle is greyed out')


# ______________________________________ A2. custom, mask on one device only

# The subtlest part of the design. Clones replicate the master's children, so two
# custom devices cannot hold different operators. Per device divergence has to
# come from the switch1 expression, and this is what proves it does.

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
	check('custom: masked device selects the masked branch',
		COMP.op('Device1/switch1').par.index.eval() == 1,
		'row 1 supplies a mask TOP')
	check('custom: unmasked device stays on the raw branch',
		COMP.op('Device2/switch1').par.index.eval() == 0,
		'row 2 has an empty mask cell, so the guard pins it to raw')
	COMP.par.Usemaskforcalibration = False


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

	for i in (1, 2, 3):
		top = COMP.op('Device{}/{}'.format(i, CLOUD))
		ok = top is not None and top.width > 0 and top.height > 0
		check('Device{} resolves a cloud through the custom select'.format(i), ok,
			'{}x{}'.format(top.width, top.height) if top else 'missing')

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

	# The one pulse path: RANSAC and ICP chained inside a single worker run. It
	# has to land where the two pulse path lands, and say so for both stages.
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
COMP.BuildDeviceSources()
for extra in [o for o in COMP.children if o.name.startswith('Device')
		and o.name[6:].isdigit() and int(o.name[6:]) > 1]:
	extra.destroy()
COMP.ResetCalibration()
check('back on kinectazure with kinect selects',
	COMP.op('Device1/in_pointcloud').OPType == 'kinectazureselectTOP')
check('customSources emptied', COMP.op('customSources').numRows == 1)
check('device clones cleared', COMP.op('Device2') is None)

failed = [n for n, ok, _ in results if not ok]
print('\n{} checks, {} failed'.format(len(results), len(failed)))
for n in failed:
	print('  FAIL {}'.format(n))


# ____________________________________________ C. the parexec1 rebuild trigger

# A Devicetype change alone must retype the template. That happens in a
# parameterexecuteDAT callback, which runs at the end of a frame rather than
# inside this script, so it has to be checked from delayed run() calls. The
# delays also let the callbacks queued by everything above drain first.

COMP.store('triggerResults', [])

STEP = ("c = op('/ProjectName/TDXDepthCameraMerger')\n"
	"c.store('triggerResults', c.fetch('triggerResults', []) + [({!r}, bool({}))])")


def schedule(code, frames):
	run(code, delayFrames=frames, fromOP=COMP)


schedule("op('/ProjectName/TDXDepthCameraMerger').par.Devicetype = 'orbbec'", 10)
schedule(STEP.format('Devicetype -> orbbec rebuilds via parexec1',
	"c.op('Device1/in_pointcloud').OPType == 'orbbecselectTOP' "
	"and c.op('Device1/in_mask') is None "
	"and c.op('Device1/switch1').par.index.mode == ParMode.CONSTANT"), 14)
schedule("op('/ProjectName/TDXDepthCameraMerger').par.Devicetype = 'kinectazure'", 18)
schedule(STEP.format('Devicetype -> kinectazure rebuilds via parexec1',
	"c.op('Device1/in_pointcloud').OPType == 'kinectazureselectTOP' "
	"and c.op('Device1/in_mask') is not None "
	"and c.op('Device1/switch1').par.index.mode == ParMode.EXPRESSION"), 22)

print('\nC. parexec1 trigger scheduled. In a few frames, read:')
print("   op('/ProjectName/TDXDepthCameraMerger').fetch('triggerResults')")
