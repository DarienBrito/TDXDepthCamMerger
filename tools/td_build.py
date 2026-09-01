"""
Build TDXDepthCamMerger 0.2.0 inside a live TouchDesigner 2025.33070.

Run from TD (MCP execute_code):

    exec(open(r'<this file>', encoding='utf-8').read())

Idempotent: destroys and rebuilds the component each time, so it can be re-run
after a crash without hand work.

Why a fresh container rather than loading the .tox directly: the 2023.12370
container COMP's own saved state crashes TD 2025.33070 when it cooks. Its 145
children are all fine, so we make a new container and copyOPs them in.
"""

import os

# Anchored on the open project rather than a machine path. This file is run with
# exec(open(...).read()), which defines no __file__, so project.folder is the only
# handle on the repo. Set REPO before the exec to build from somewhere else.
REPO = globals().get('REPO') or project.folder.replace(os.sep, '/')
SRC = REPO + '/src'
BUILD = REPO + '/tools/build'
TOX = BUILD + '/control.tox'

# Fail loud rather than build the wrong thing: a wrong REPO used to install stale
# sources silently.
if not os.path.isfile(SRC + '/worker.py'):
	raise RuntimeError(
		'no sources under {}. Open the project from the repo root, or set REPO '
		'before the exec.'.format(SRC))

HOST = op('/ProjectName')
NAME = 'TDXDepthCamMerger'
SHORTCUT = 'TDXMerger'
OLD_SHORTCUT = 'TDAzureMerger'
PYEXE = os.environ.get('TDX_PYTHON_EXE') or 'D:/anaconda3/envs/td/python.exe'

VERSION = '0.2.0'
AUTHOR = 'Darien Brito'

report = []


def note(msg):
	report.append(msg)


# ____________________________________________________ 1. fresh container

for leftover in ('probe', 'exttest', 'mtx_test', NAME, '_stage'):
	if HOST.op(leftover):
		HOST.op(leftover).destroy()

stage = HOST.create(baseCOMP, '_stage')
stage.nodeX, stage.nodeY = -600, 600
stage.loadTox(TOX)
old = stage.op('DBLib_TDAzureMerger')
note('loaded source tox: {} descendants'.format(len(old.findChildren(maxDepth=99))))

comp = HOST.create(containerCOMP, NAME)
comp.nodeX, comp.nodeY = 0, 400
comp.par.parentshortcut = SHORTCUT
comp.par.w, comp.par.h = 1920, 1080
comp.par.hmode, comp.par.vmode = 'fill', 'fill'

# copyOPs preserves connections among the copied set; copy() one at a time does not.
comp.copyOPs(list(old.children))
stage.destroy()
note('copied {} descendants into the new container'.format(len(comp.findChildren(maxDepth=99))))


# ____________________________________________________ 2. rename Azure -> Device

renamed = []
for child in list(comp.children):
	if child.name.startswith('Azure') and child.name[5:].isdigit():
		child.name = 'Device' + child.name[5:]
		renamed.append(child.name)
for child in list(comp.op('World').children):
	if child.name.startswith('AzureCam'):
		child.name = 'DeviceCam' + child.name[8:]
		renamed.append(child.name)
note('renamed: {}'.format(', '.join(renamed) or 'none'))


# ______________________________ 3. repoint every reference to the new names

SUBS = [(OLD_SHORTCUT, SHORTCUT), ('AzureCam', 'DeviceCam'), ('Azure', 'Device')]
touched = []
for o in comp.findChildren(maxDepth=99) + [comp]:
	for p in o.pars():
		try:
			if p.mode == ParMode.EXPRESSION and p.expr:
				new = p.expr
				for a, b in SUBS:
					new = new.replace(a, b)
				if new != p.expr:
					touched.append('{}.{} (expr)'.format(o.name, p.name))
					p.expr = new
			elif p.mode == ParMode.CONSTANT and isinstance(p.val, str) and 'Azure' in p.val:
				new = p.val
				for a, b in SUBS:
					new = new.replace(a, b)
				if new != p.val:
					touched.append('{}.{} = {}'.format(o.name, p.name, new))
					p.val = new
		except Exception:
			pass
note('repointed {} parameter references'.format(len(touched)))
for t in touched:
	note('    ' + t)


# ____________________________________________________ 4. custom parameters
#
# Declared here rather than replayed from the old custom_pars.json dump, which
# is deleted: it shipped the original machine's Kinect serials as the defaults of
# Devices and Ids, carried seven parameters 0.2.0 no longer has, and split one
# page of state across a json file and forty lines of appends. Every runtime
# parameter now starts empty and is filled by the extensions.
#
# Each row is (name, label, kind, options).
#   section  draws a divider above the parameter
#   readOnly is display only, script still writes it
# Reading order per page: what you set, then what you press, then what comes back.

PAGES = (
	('Setup', (
		('Inputsop', 'Inputs op', 'OP', {}),
		('Devicetype', 'Device type', 'Menu', {
			'menuNames': ('kinectazure', 'orbbec', 'custom'),
			'menuLabels': ('Kinect Azure', 'Orbbec', 'Custom TOPs')}),
		('Gatherdevices', 'Gather devices', 'Pulse', {}),
		('Numberofdevices', 'Number of devices', 'Int', {'readOnly': True}),
		('Devices', 'Devices', 'Str', {'readOnly': True}),
		('Pythonexe', 'Python exe', 'File', {'val': PYEXE, 'section': True}),
		('Checkworker', 'Check worker', 'Pulse', {}),
		('Open3dstatus', 'Worker status', 'Str', {'readOnly': True}),
	)),
	('Calibrate', (
		('Referencedevice', 'Reference device', 'Int', {'val': 1}),
		('Specifypair', 'Specify pair', 'Int', {'size': 2, 'val': (1, 2)}),
		('Ids', 'IDs', 'Str', {'readOnly': True}),
		('Calibrate', 'Calibrate', 'Pulse', {'section': True}),
		('Refine', 'Refine', 'Pulse', {}),
		('Rebuildchain', 'Rebuild chain', 'Pulse', {}),
		('Resetcalibration', 'Reset calibration', 'Pulse', {}),
		('Laststatus', 'Last status', 'Str', {'readOnly': True, 'section': True}),
		('Lastfitness', 'Last fitness', 'Str', {'readOnly': True}),
		('Lastrmse', 'Last RMSE', 'Str', {'readOnly': True}),
		('Lastcorrespondences', 'Last correspondences', 'Int', {'readOnly': True}),
	)),
	('Registration', (
		# Chained is the default: calibrating from scratch wants both stages, and
		# doing them in one worker run costs one process start instead of two.
		('Mode', 'Mode', 'Menu', {
			'val': 'globalThenIcp',
			'menuNames': ('globalThenIcp', 'globalRegistration', 'table'),
			'menuLabels': ('Global + ICP refine', 'Global registration only',
				'From table DAT')}),
		('Presetmatrixdat', 'Preset matrix DAT', 'DAT', {}),
		('Voxelsize', 'Voxel size (m)', 'Float', {'val': 0.05, 'section': True}),
		('Refinevoxel', 'Refine voxel (m)', 'Float', {'val': 0.01}),
		('Maxrange', 'Max range (m)', 'Float', {'val': 0.0}),
		('Usecoloricp', 'Use coloured ICP', 'Toggle', {'section': True}),
		('Usemaskforcalibration', 'Use mask for calibration', 'Toggle', {}),
		('Seed', 'RANSAC seed', 'Int', {'val': -1}),
	)),
	('About', (
		('Readme', 'Readme', 'Pulse', {}),
		('Support', 'Support', 'Pulse', {}),
		('Website', 'Website', 'Pulse', {}),
		('Author', 'Author', 'Str', {'val': AUTHOR, 'readOnly': True,
			'section': True}),
		('Version', 'Version', 'Str', {'val': VERSION, 'readOnly': True}),
		('Open3dversion', 'Open3D version', 'Str', {'readOnly': True}),
		('Pythonversion', 'Python version', 'Str', {'readOnly': True}),
	)),
)

APPEND = {'DAT': 'appendDAT', 'File': 'appendFile', 'Float': 'appendFloat',
	'Int': 'appendInt', 'Menu': 'appendMenu', 'OP': 'appendOP',
	'Pulse': 'appendPulse', 'Str': 'appendStr', 'Toggle': 'appendToggle'}

for pagename, entries in PAGES:
	page = comp.appendCustomPage(pagename)
	for name, label, kind, opt in entries:
		kw = {'label': label}
		if opt.get('size'):
			kw['size'] = opt['size']
		group = getattr(page, APPEND[kind])(name, **kw)
		if 'menuNames' in opt:
			group[0].menuNames = list(opt['menuNames'])
			group[0].menuLabels = list(opt['menuLabels'])
		if 'val' in opt:
			vals = opt['val'] if opt.get('size') else (opt['val'],)
			for par, val in zip(group, vals):
				par.val = val
		if opt.get('readOnly'):
			for par in group:
				par.readOnly = True
		if opt.get('section'):
			group[0].startSection = True
note('custom parameters: {} across {} pages'.format(
	len(comp.customPars), len(PAGES)))


# ____________________________________________________ 5. tables and DATs

def table(name, rows, x, y):
	if comp.op(name):
		comp.op(name).destroy()
	t = comp.create(tableDAT, name)
	t.nodeX, t.nodeY = x, y
	t.clear()
	for r in rows:
		t.appendRow(r)
	return t


table('deviceTypes', [
	['type', 'cameraop_name', 'selectop', 'image_pointcloud', 'image_color', 'image_mask', 'devicepar'],
	['kinectazure', 'kinectazure', 'kinectazureselectTOP', 'pointcloud', 'color', 'playerindex', 'sensor'],
	['orbbec', 'orbbec', 'orbbecselectTOP', 'pointcloud', 'color', '', 'device'],
	['custom', '', 'selectTOP', '', '', '', ''],
], -600, -200)

table('customSources', [['name', 'pointcloud', 'color', 'mask']], -600, -320)
table('calibrationData', [], -600, -80)

for old_ in ('intermediaryMatrix',):
	if comp.op(old_):
		comp.op(old_).destroy()
		note('removed dead {}'.format(old_))


def installDat(name, path, x, y, language='python'):
	dat = comp.op(name) or comp.create(textDAT, name)
	dat.text = open(path, encoding='utf-8').read()
	dat.nodeX, dat.nodeY = x, y
	try:
		dat.par.language = language
	except Exception:
		pass
	return dat


# control.tox still carries the root extension under its 0.0.3 name. Before the
# rename the install landed on that DAT and overwrote it; now it would survive
# as a duplicate and push the descendant count to 138.
legacy = comp.op('extTDAzureMerger')
if legacy is not None:
	legacy.destroy()
	note('removed the 0.0.3 extTDAzureMerger DAT copied from control.tox')

installDat('extTDXDepthCamMerger', SRC + '/extTDXDepthCamMerger.py', -900, 200)
installDat('extUtilities', SRC + '/extUtilities.py', -900, 80)
installDat('workerSource', SRC + '/worker.py', -900, -40)
note('installed extension, utilities and worker source from {}'.format(SRC))
note('python exe: {}'.format(PYEXE))

comp.op('parexec1').text = '''# Routes every root parameter pulse: the work goes to the extensions, the three
# About pulses go to the system browser. There is no webrender TOP in the
# component, so opening a link starts no CEF process inside TouchDesigner.

import webbrowser

LINKS = {
	'Readme': 'https://github.com/DarienBrito/TDAzureMerger',
	'Support': 'https://www.patreon.com/c/darienbrito',
	'Website': 'https://www.darienbrito.com',
}


def onValueChange(par, prev):
	comp = parent()
	if par.name in ('Specifypair1', 'Specifypair2', 'Devices'):
		comp.SetIds()
	elif par.name == 'Devicetype':
		# Retype the template right away rather than leaving Kinect selects in
		# place until the next Gather devices pulse.
		comp.BuildDeviceSources()
	return


def onPulse(par):
	comp = parent()
	name = par.name

	if name == 'Calibrate':
		comp.Calibrate(pair=comp.GetPair(), mode=comp.par.Mode.eval())
	elif name == 'Refine':
		comp.Refine(pair=comp.GetPair())
	elif name == 'Gatherdevices':
		comp.GatherDevices()
	elif name == 'Rebuildchain':
		comp.RebuildChain()
	elif name == 'Resetcalibration':
		comp.ResetCalibration()
	elif name == 'Checkworker':
		comp.CheckWorker()
	elif name in LINKS:
		webbrowser.open(LINKS[name])
	return
'''
comp.op('parexec1').par.ops = comp
comp.op('parexec1').par.pars = '*'


# ____________________________________________________ 6. extensions

# The Extension Object parameter holds python CODE AS A CONSTANT STRING.
# Setting it as an expression silently fails to bind.
comp.par.ext0object.mode = ParMode.CONSTANT
comp.par.ext0object.val = "op('./extTDXDepthCamMerger').module.extTDXDepthCamMerger(me)"
comp.par.ext0promote = True
comp.par.ext1object.mode = ParMode.CONSTANT
comp.par.ext1object.val = "op('./extUtilities').module.extUtilities(me)"
comp.par.ext1promote = True
comp.par.initextonstart = True
comp.par.reinitextensions.pulse()


# ____________________________________________________ 7. per device COMP

device = comp.op('Device1')

# The blanket Azure -> Device rename above also rewrote this device's extension
# reference, so the DAT and its class have to follow.
azure = device.op('extAzure')
if azure:
	azure.destroy()
devExt = device.create(textDAT, 'extDevice')
devExt.text = open(SRC + '/extDevice.py', encoding='utf-8').read()
devExt.nodeX, devExt.nodeY = -275, 175
try:
	devExt.par.language = 'python'
except Exception:
	pass
device.par.ext0object.mode = ParMode.CONSTANT
device.par.ext0object.val = "op('./extDevice').module.extDevice(me)"
device.par.ext0promote = True
device.par.reinitextensions.pulse()

if hasattr(device.par, 'Useplayer'):
	device.par.Useplayer.expr = 'parent().par.Usemaskforcalibration'
note('rebuilt Device1 extension as extDevice')

# The 4th channel of the point cloud TOP is alpha/validity, NOT a homogeneous w.
# Measured in 2025.33070: feeding it as w made alpha 0.5 shift X by -0.50 instead
# of +1.00, and alpha 0 collapsed the whole cloud onto the origin.
device.op('glsl2_pixel').text = '''// Applies this device's composed transform to its point cloud on the GPU.
//
// w MUST be 1.0. The 4th channel of a point cloud TOP is alpha/validity, not a
// homogeneous coordinate; using it as w scales the translation column. Measured
// in TD 2025.33070 with a +1.0 X translation: alpha 1.0 gave +1.00 (correct),
// alpha 0.5 gave -0.50, alpha 0.0 collapsed everything onto the origin.

uniform int uShow;
uniform mat4 mTransformMatrix;

out vec4 fragColor;

void main()
{
\tivec2 xy = ivec2(gl_FragCoord.xy);
\tvec4 texel = texelFetch(sTD2DInputs[0], xy, 0);

\tvec3 position = (mTransformMatrix * vec4(texel.xyz, 1.0)).xyz;

\t// Binary mask rather than multiplying by a possibly fractional alpha, so a
\t// valid point is never partially scaled toward the origin.
\tfloat keep = step(0.5, texel.a) * float(uShow);

\tfragColor = TDOutputSwizzle(vec4(position * keep, texel.a * float(uShow)));
}
'''

# The compute shader was TouchDesigner's untouched example: it writes solid
# white into a compute output that nothing reads.
device.op('glsl2').par.computedat = ''
if device.op('glsl2_compute'):
	device.op('glsl2_compute').destroy()
note('fixed glsl2_pixel homogeneous w and uShow; removed the unused compute shader')

# Build the source TOPs through the same code path the Gather devices pulse uses,
# rather than inheriting the 0.0.3 hardcoded Kinect selects from the old tox.
# Called on the module directly because bound extensions are only live next frame.
utils = comp.op('extUtilities').module.extUtilities(comp)
built = utils.BuildDeviceSources(device)
note('built device sources for "{}": {}'.format(
	comp.par.Devicetype.eval(),
	', '.join('{} ({})'.format(n, o.OPType) for n, o in sorted(built.items()))))


# _________________________________ 8. strip inherited dead weight, add outputs

world = comp.op('World')

# The DeviceCam frustums are shaded by phong1, which is why this component
# carries a lightCOMP and an ambientlightCOMP. Dropping them for a constantMAT
# or a wireframeMAT was tried and rendered worse: TD's camera gizmo is a dense
# mesh, so unlit it collapses to a silhouette and in wireframe it turns to
# scribble. The 32 operators inside light1 are TD's own gizmo geometry: one
# file load at startup, nothing per frame. Keeping the lights.
# 'World/*' swept all twelve World children into the light list; name the two.
comp.op('render1').par.lights = 'World/light1 World/ambient1'

stripped = []

# The whole 'actions' baseCOMP goes: it held a Readme containerCOMP wrapping a
# webrenderTOP left active (a CEF process per instance, on load) and a second
# parameterexecuteDAT that only called webbrowser.open. The root parexec1 above
# already sees every root parameter, so it routes the About links too.
if comp.op('actions'):
	comp.op('actions').destroy()
	stripped.append('actions (COMP, was a second parexec + a CEF Readme)')

# The DeviceCam replicator counted rows in an opfindDAT that rescanned the
# network every cook, to learn a number the component already knows.
rep = world.op('replicator1')
rep.par.template = ''
rep.par.method = 'bynum'
rep.par.numreplicants.expr = 'parent.{}.par.Numberofdevices'.format(SHORTCUT)
rep.par.repsuffixstart = 1
# replicator1_callbacks was a stub: onReplicate passed, and onRemoveReplicant
# reimplemented the default. Verified in 2025.33070 that clones are still
# created and destroyed correctly with the callbacks parameter empty.
rep.par.callbacks = ''
for dead in ('opfind1', 'opfind1_callbacks', 'World/replicator1_callbacks'):
	if comp.op(dead):
		comp.op(dead).destroy()
		stripped.append(dead)

# infoDAT on glsl2, referenced by no parameter and no DAT. One per device.
if device.op('glsl2_info'):
	device.op('glsl2_info').destroy()
	stripped.append('Device1/glsl2_info')

# 'mtx' was a null DAT passing transformMatrix straight through to the shader's
# matrix uniform. The uniform takes the table directly. One operator per device.
if device.op('mtx'):
	device.op('glsl2').par.matrix0value = 'transformMatrix'
	device.op('mtx').destroy()
	stripped.append('Device1/mtx (null DAT, uniform now reads transformMatrix)')

# UI/Viz reached the render through a selectTOP holding an ABSOLUTE path
# (/ProjectName/TDXDepthCamMerger/bg), which breaks the moment the .tox is
# dropped into a project with any other root name. A panel's Background TOP
# parameter resolves an operator across networks, so it points at bg itself.
viz = comp.op('UI/Viz')
if viz and viz.op('select2'):
	viz.par.top.val = viz.relativePath(comp.op('bg'))
	for dead in ('bg', 'select2'):
		viz.op(dead).destroy()
	stripped.append('UI/Viz/select2 + UI/Viz/bg (absolute path removed)')

# Active duplicates Show, which is what actually reaches the shader as uShow.
# Parentdevice was written by createDevices and read by nothing; the real parent
# is the one recorded in calibrationData from the calibration pair.
for dead in ('Active', 'Parentdevice'):
	par = getattr(device.par, dead, None)
	if par is not None:
		par.destroy()
		stripped.append('Device1.{} (par)'.format(dead))
note('stripped: {}'.format(', '.join(stripped) or 'nothing'))

# Give the component real outputs. Until now the merged cloud could only be
# reached by an absolute path into World. An outTOP resolves `selecttop` across
# network boundaries, so this needs no select TOPs of its own.
for name, source, order, y in (
		('out_points', 'World/mergedPointClouds', 0, 0),
		('out_colors', 'World/mergedColors', 1, -120)):
	out = comp.op(name) or comp.create(outTOP, name)
	out.pars('selecttop')[0].val = source
	out.par.connectorder = order
	out.nodeX, out.nodeY = 1225, y
note('added out_points and out_colors')


# ____________________________________________________ 9. annotations

# Comments are free: every operator carries one, readable in the network editor
# and in the OP dialog. Boxes are not. TD 2025 has no lightweight networkBox any
# more, and one annotateCOMP is 24 internal operators and ~1.9 KB of .tox, so
# there are three of them, over the three regions the network is navigated by,
# rather than one per cluster. Measured 2026-09-01: 132 ops / 27.0 KB bare,
# 204 ops / 36.2 KB with the three boxes and the comments below.

COMMENTS = {
	'extTDXDepthCamMerger': 'Root extension: Calibrate, Refine, RebuildChain, '
		'ResetCalibration, CheckWorker. Imports numpy, never open3d.',
	'extUtilities': 'Device discovery, per device source TOPs, clone management.',
	'workerSource': 'worker.py. Written to temp and run by the Setup python as a '
		'subprocess. Importing open3d inside TouchDesigner crashes the process, '
		'which is why all the Open3D work happens out there.',
	'deviceTypes': 'Camera registry. One row per supported camera: what operator '
		'to make, which image to select, what parameter holds the serial. A new '
		'camera is a new row.',
	'calibrationData': 'Source of truth. One row per device: its parent and its '
		'RAW pairwise matrix, plus the quality of that fit.',
	'customSources': 'Device type "custom" only: explicit TOP paths, one row per '
		'device.',
	'parexec1': 'Routes every root parameter. Pulses go to the extensions, the '
		'three About links go to the system browser.',
	'Device1': 'Clone master. Device2..N are clones made by the Gather devices '
		'pulse. transformMatrix is cloneImmune, so each device keeps its own.',
	'World': 'The merged cloud and one DeviceCam frustum per device. replicator1 '
		'counts by Numberofdevices.',
	'cam1': 'ArcBall viewport navigation.',
	'UI': 'Viewport panel plus the parameter COMP.',
	'render1': 'Viewport render.',
	'bg': 'Viewport image. UI/Viz reads it through its Background TOP parameter.',
	'out_points': 'Component output: merged point positions, straight out of World.',
	'out_colors': 'Component output: merged colours, straight out of World.',
}
for name, text in COMMENTS.items():
	if comp.op(name):
		comp.op(name).comment = text

DEVICE_COMMENTS = {
	'in_pointcloud': 'Built from the deviceTypes row, not hardcoded.',
	'in_color': 'Built from the deviceTypes row, not hardcoded.',
	'in_mask': 'Built only when the camera has a mask source. Orbbec has none.',
	'thresh1': 'Mask branch: player index to a 0/1 matte.',
	'switch1': 'Raw cloud or masked cloud, per Usemaskforcalibration.',
	'null_sourcePointcloud': 'What Calibrate samples and dumps to .npy.',
	'glsl2': 'Applies the composed matrix on the GPU. w is forced to 1.0.',
	'transformMatrix': 'This device COMPOSED into the reference frame. The only '
		'table the shader and the frustum read.',
	'null_pointCloud': 'What the renderer reads.',
}
for name, text in DEVICE_COMMENTS.items():
	if device.op(name):
		device.op(name).comment = text
note('commented {} operators'.format(len(COMMENTS) + len(DEVICE_COMMENTS)))

# Boxes are sized from the operators they hold, so moving a node cannot leave a
# box behind. Padding leaves room for the title bar at the top.
BOXES = (
	('Code and state', ('extTDXDepthCamMerger', 'extUtilities', 'workerSource',
		'deviceTypes', 'calibrationData', 'customSources'),
		'The component in text. The three DATs on the left are installed from '
		'src/ by tools/td_build.py; editing them in here is reverted by the next '
		'build. The three tables are its state.'),
	('Devices', ('Device1',),
		'One COMP per camera, cloned from Device1: source TOPs, an optional mask '
		'branch, and the GPU transform.'),
	('Render and outputs', ('World', 'cam1', 'UI', 'render1', 'bg', 'out_points',
		'out_colors'),
		'The merged cloud, the viewport that shows it, and the two outTOPs that '
		'hand it downstream. No Python runs per frame in here.'),
)

for old in comp.findChildren(type=annotateCOMP, maxDepth=1):
	old.destroy()

for title, members, body in BOXES:
	ops = [comp.op(m) for m in members if comp.op(m)]
	# 20 sideways keeps the Devices box clear of the Render box, which sits 65
	# units to its right; 85 above leaves room for the title bar.
	left = min(o.nodeX for o in ops) - 20
	right = max(o.nodeX + o.nodeWidth for o in ops) + 20
	bottom = min(o.nodeY for o in ops) - 30
	top = max(o.nodeY + o.nodeHeight for o in ops) + 85
	a = comp.create(annotateCOMP)
	a.par.Mode = 'networkbox'
	a.par.Titletext = title
	a.par.Bodytext = body
	a.par.Bodyfontsize = 14
	a.par.Backcolorr, a.par.Backcolorg, a.par.Backcolorb = 0.11, 0.12, 0.15
	a.par.Backcoloralpha = 0.6
	a.nodeX, a.nodeY = left, bottom
	a.nodeWidth, a.nodeHeight = right - left, top - bottom
note('annotated {} regions'.format(len(BOXES)))


comp.cook(force=True)
errs = [(o.path.split(NAME + '/')[-1], o.errors().replace('\n', ' ')[:100])
	for o in comp.findChildren(maxDepth=99) if o.errors()]
note('errors after build: {}'.format(len(errs)))
for p, e in errs:
	note('    E {} -> {}'.format(p, e))


# ____________________________________________________ 10. export the artefact

# The shipped .tox must not carry this machine's interpreter path: a user opening
# it would find a python.exe that does not exist on their disk, and the README
# already walks them through setting their own. The master keeps PYEXE, because
# tools/td_test_devicesources.py drives a real registration through it.
keep = comp.par.Pythonexe.eval()
comp.par.Pythonexe = ''
comp.save(REPO + '/' + NAME + '.tox')
comp.par.Pythonexe = keep
note('exported {}/{}.tox, Pythonexe cleared in the artefact only'.format(REPO, NAME))

print('\n'.join(report))
print('\nBUILD DONE. Extensions bind on the next frame; verify then.')
