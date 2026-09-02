"""
Build TDXDepthCamMerger 0.3.0 inside a running TouchDesigner 2025.33070.

Run from TD (MCP execute_code):

    exec(open(r'<this file>', encoding='utf-8').read())

Safe to run again and again: it destroys and rebuilds the component each time,
so a crash needs no hand repair.

Everything is copied into a FRESH container rather than loaded straight from
the old .tox, whose saved container crashes this TD build when it cooks. Its
children are all fine, so they are copied across.
"""

import os

# Taken from the open project rather than a fixed path: this file is run with
# exec(), which defines no __file__, so the project folder is the only handle on
# the repo. Set REPO before the exec to build from somewhere else.
REPO = globals().get('REPO') or project.folder.replace(os.sep, '/')
SRC = REPO + '/src'
BUILD = REPO + '/tools/build'
TOX = BUILD + '/control.tox'

# Stop rather than build the wrong thing: a wrong REPO would quietly install
# stale sources.
if not os.path.isfile(SRC + '/worker.py'):
	raise RuntimeError(
		'no sources under {}. Open the project from the repo root, or set REPO '
		'before the exec.'.format(SRC))

HOST = op('/ProjectName')
NAME = 'TDXDepthCamMerger'
SHORTCUT = 'TDXMerger'
OLD_SHORTCUT = 'TDAzureMerger'
PYEXE = os.environ.get('TDX_PYTHON_EXE') or 'D:/anaconda3/envs/td/python.exe'

VERSION = '0.3.0'
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

# copyOPs keeps the wires between the copied operators; copying one at a time
# drops them.
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
# Written out here rather than replayed from a dump, so the shipped component
# carries nothing from this machine. Every parameter filled in at runtime starts
# empty and is written by the extensions.
#
# Each row is (name, label, kind, options).
#   section  draws a divider above the parameter
#   readOnly is display only, the script still writes it
# Order on a page: what you set, then what you press, then what comes back.

PAGES = (
	('Setup', (
		('Inputsop', 'Inputs op', 'OP', {}),
		('Devicetype', 'Device type', 'Menu', {
			'menuNames': ('kinectazure', 'orbbec', 'zed', 'custom'),
			'menuLabels': ('Kinect Azure', 'Orbbec', 'ZED', 'Custom TOPs')}),
		('Gatherdevices', 'Gather devices', 'Pulse', {}),
		('Numberofdevices', 'Number of devices', 'Int', {'readOnly': True}),
		('Devices', 'Devices', 'Str', {'readOnly': True}),
		('Pythonexe', 'Python exe', 'File', {'val': PYEXE, 'section': True}),
		('Checkworker', 'Check worker', 'Pulse', {}),
		# Worker status says only OK, or what went wrong. The two versions get
		# their own fields beside it, so they can be read at a glance and used
		# in an expression.
		('Open3dstatus', 'Worker status', 'Str', {'readOnly': True}),
		('Open3dversion', 'Open3D version', 'Str', {'readOnly': True}),
		('Pythonversion', 'Python version', 'Str', {'readOnly': True}),
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
		# Both stages at once is the default: calibrating from scratch wants
		# both, and one run starts one process instead of two.
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


# sourcepar is the parameter on the SELECT that names the camera TOP. Kinect and
# Orbbec call it `top`, ZED calls it `zedtop`. ZED's mask image is deliberately
# empty: it holds body IDs and needs body tracking on through a ZED CHOP, and the
# mask threshold here was tuned against the Kinect's player index.
table('deviceTypes', [
	['type', 'cameraop_name', 'selectop', 'image_pointcloud', 'image_color', 'image_mask',
		'devicepar', 'sourcepar'],
	['kinectazure', 'kinectazure', 'kinectazureselectTOP', 'pointcloud', 'color', 'playerindex',
		'sensor', 'top'],
	['orbbec', 'orbbec', 'orbbecselectTOP', 'pointcloud', 'color', '', 'device', 'top'],
	['zed', 'zed', 'zedselectTOP', 'pointcloud', 'color', '', 'camera', 'zedtop'],
	['custom', '', 'selectTOP', '', '', '', '', 'top'],
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


# control.tox carries the root extension under an older name. Destroy it, or it
# stays behind as a duplicate.
legacy = comp.op('extTDAzureMerger')
if legacy is not None:
	legacy.destroy()
	note('removed the 0.0.3 extTDAzureMerger DAT copied from control.tox')

installDat('extTDXDepthCamMerger', SRC + '/extTDXDepthCamMerger.py', -900, 200)
installDat('extUtilities', SRC + '/extUtilities.py', -900, 80)
installDat('workerSource', SRC + '/worker.py', -900, -40)
note('installed extension, utilities and worker source from {}'.format(SRC))
note('python exe: {}'.format(PYEXE))

comp.op('parexec1').text = '''# Handles every parameter on the component: the work goes to the extensions, the
# three About buttons open a link in the system browser. Nothing here loads a
# web page inside TouchDesigner.

import webbrowser

LINKS = {
	'Readme': 'https://github.com/DarienBrito/TDXDepthCamMerger',
	'Support': 'https://www.patreon.com/c/darienbrito',
	'Website': 'https://www.darienbrito.com',
}


def onValueChange(par, prev):
	comp = parent()
	if par.name in ('Specifypair1', 'Specifypair2', 'Devices'):
		comp.SetIds()
	elif par.name == 'Devicetype':
		# Rebuild the template straight away rather than leaving the old camera
		# selects in place until the next Gather devices pulse. RebuildDevices,
		# not BuildDeviceSources: rebuilding destroys the template's operators,
		# and every clone has to be copied again afterwards.
		comp.RebuildDevices()
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

# The Extension Object parameter holds python code as a plain string. Set as an
# expression it binds nothing and says nothing.
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

# The rename above also rewrote this device's extension reference, so the DAT
# and its class follow.
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

# No shader here. The calibration is applied by a transform POP reading the
# transformMatrix table, and BuildDeviceSources destroys whatever the old chain
# left behind.
if device.op('glsl2_compute'):
	device.op('glsl2_compute').destroy()

# Build the source TOPs and the point chain with the same code the Gather
# devices pulse uses. Called on the module directly, because an extension
# attached to a COMP only goes live on the next frame.
utilsModule = comp.op('extUtilities').module
utils = utilsModule.extUtilities(comp)
built = utils.BuildDeviceSources(device)
note('built device sources for "{}": {}'.format(
	comp.par.Devicetype.eval(),
	', '.join('{} ({})'.format(n, o.OPType) for n, o in sorted(built.items()))))
note('device POP chain: {}'.format(' -> '.join(
	'{} ({})'.format(n, device.op(n).OPType)
	for n in utilsModule.POP_NAMES if device.op(n))))


# _________________________________ 8. strip inherited dead weight, add outputs

world = comp.op('World')

# The little camera shapes are shaded by phong1, which is why the component
# carries a light and an ambient light. Unlit they turn into flat silhouettes.
# The operators inside light1 are TD's own gizmo shapes: loaded once, nothing
# per frame.
# Name the two lights: a wildcard would sweep in every child of World.
comp.op('render1').par.lights = 'World/light1 World/ambient1'

# Put phong1 in the row with the two lights it works with, where it reads.
world.op('phong1').nodeX, world.op('phong1').nodeY = 1125, 225

stripped = []

# The whole "actions" COMP goes: it held a web page that loaded a browser
# process on startup, and a second parameter handler. parexec1 above already
# sees every parameter, so it opens the About links too.
if comp.op('actions'):
	comp.op('actions').destroy()
	stripped.append('actions (COMP, was a second parexec + a CEF Readme)')

# Take the number of camera gizmos straight from the parameter, rather than
# searching the network every cook for a number the component already knows.
rep = world.op('replicator1')
rep.par.template = ''
rep.par.method = 'bynum'
rep.par.numreplicants.expr = 'parent.{}.par.Numberofdevices'.format(SHORTCUT)
rep.par.repsuffixstart = 1
# Its callbacks DAT did nothing the default does not already do. Clones are
# still made and removed correctly with the parameter left empty.
rep.par.callbacks = ''
for dead in ('opfind1', 'opfind1_callbacks', 'World/replicator1_callbacks'):
	if comp.op(dead):
		comp.op(dead).destroy()
		stripped.append(dead)

# BuildDeviceSources above already clears out anything left over inside a
# device, so there is nothing to strip per device here.

# UI/Viz reached the render through a select holding a full path from the root,
# which breaks the moment the .tox is dropped into a project with another name.
# A panel's Background TOP parameter finds an operator across networks, so it
# points straight at bg.
viz = comp.op('UI/Viz')
if viz and viz.op('select2'):
	viz.par.top.val = viz.relativePath(comp.op('bg'))
	for dead in ('bg', 'select2'):
		viz.op(dead).destroy()
	stripped.append('UI/Viz/select2 + UI/Viz/bg (absolute path removed)')

# Active did the same job as Show, which is what really gates the merged cloud.
# Parentdevice was written and never read: the real parent is the one recorded
# in calibrationData.
for dead in ('Active', 'Parentdevice'):
	par = getattr(device.par, dead, None)
	if par is not None:
		par.destroy()
		stripped.append('Device1.{} (par)'.format(dead))

# The two layout TOPs tiled every device's cloud and colour into big textures.
# A merge POP gathers the real points instead, through a list of POP parameters
# rather than wires, which is how a changing number of devices reaches one
# output. extUtilities.WireMerge fills the list in.
for dead in ('layout1', 'layout2', 'mergedPointClouds', 'mergedColors'):
	if world.op(dead):
		world.op(dead).destroy()
		stripped.append('World/{}'.format(dead))

merge = world.op('mergePOP') or world.create(mergePOP, 'mergePOP')
# Sits where the layout did, so World still reads left to right.
merge.nodeX, merge.nodeY = 175, -25

# A select POP rather than a null, because it can pick attributes: valid and
# maskv are only used to decide what to keep, so they stop here. The merged
# cloud carries P and Color.
merged = world.op('merged') or world.create(selectPOP, 'merged')
merged.par.pop = 'mergePOP'
merged.par.pointattrscope = 'P Color'
merged.nodeX, merged.nodeY = 350, -25

# geo_pc renders the merged cloud itself, pulled in by a select POP with its
# render and display flags on. The material is still World/pointsprite1.
geo = world.op('geo_pc')
geo.par.instanceop = ''
geo.par.instancing = False
for child in list(geo.children):
	child.destroy()
pc = geo.create(selectPOP, 'pc')
pc.par.pop = '../merged'
pc.render = True
pc.display = True
note('World merges {} -> merged (P Color) -> geo_pc/pc'.format(merge.name))

# Give the component a real output connector. An out POP can point at a POP in
# another network, so it needs no select of its own.
for dead in ('out_points', 'out_colors'):
	if comp.op(dead):
		comp.op(dead).destroy()
		stripped.append(dead + ' (outTOP, superseded by out_pop)')
out = comp.op('out_pop') or comp.create(outPOP, 'out_pop')
out.par.selectpop = 'World/merged'
out.par.connectorder = 0
out.nodeX, out.nodeY = 1225, 0
note('stripped: {}'.format(', '.join(stripped) or 'nothing'))
note('added out_pop')

# Point the merge at whatever devices exist right now. Every later Gather devices
# pulse does this again, from the same method.
utils.WireMerge()
note('merge wired to {} device(s)'.format(merge.seq.input.numBlocks))


# ____________________________________________________ 9. annotations

# Each box costs 24 operators and about 1.9 KB in the .tox, so there are three
# of them, one per region of the network, rather than one per cluster.

# Comments on single operators clutter the network view, so there are none. The
# explanations live in STRUCTURE.md and in the three boxes below. Clear any left
# by an earlier build, or a stale one would survive forever.
cleared = 0
for child in comp.findChildren(includeUtility=True):
	if child.comment:
		child.comment = ''
		cleared += 1
if comp.comment:
	comp.comment = ''
	cleared += 1
note('cleared {} leftover operator comments'.format(cleared))

# Boxes are sized from the operators they hold, so moving a node cannot leave a
# box behind. Padding leaves room for the title bar at the top.
BOXES = (
	('Code and state', ('extTDXDepthCamMerger', 'extUtilities', 'workerSource',
		'deviceTypes', 'calibrationData', 'customSources'),
		'The component in text. The three DATs on the left are installed from '
		'src/ by tools/td_build.py; editing them in here is reverted by the next '
		'build. The three tables are its state.'),
	('Devices', ('Device1',),
		'One COMP per camera, cloned from Device1. Three source TOPs, because '
		'that is what the camera SDKs give; everything past them is a POP.'),
	('Render and outputs', ('World', 'cam1', 'UI', 'render1', 'bg', 'out_pop'),
		'The merged point cloud, the viewport that shows it, and the outPOP that '
		'hands it downstream. No Python runs per frame in here.'),
)

for old in comp.findChildren(type=annotateCOMP, maxDepth=1):
	old.destroy()

for title, members, body in BOXES:
	ops = [comp.op(m) for m in members if comp.op(m)]
	# 20 sideways keeps neighbouring boxes apart; 85 above leaves room for the
	# title bar.
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
# A POP says "No input POP" until whatever feeds it has cooked, so walk the
# chain once before looking for errors. Cooking the container does not reach it.
for o in comp.findChildren(type=POP, maxDepth=99):
	o.cook(force=True)
errs = [(o.path.split(NAME + '/')[-1], o.errors().replace('\n', ' ')[:100])
	for o in comp.findChildren(maxDepth=99) if o.errors()]
note('errors after build: {}'.format(len(errs)))
for p, e in errs:
	note('    E {} -> {}'.format(p, e))


# ____________________________________________________ 10. export the artefact

# The shipped .tox must not carry this machine's python path: a user would open
# it and find a python.exe that is not on their disk, and the README walks them
# through setting their own. The working file keeps it, because the in-TD test
# runs a real registration through it.
keep = comp.par.Pythonexe.eval()
comp.par.Pythonexe = ''
comp.save(REPO + '/' + NAME + '.tox')
comp.par.Pythonexe = keep
note('exported {}/{}.tox, Pythonexe cleared in the artefact only'.format(REPO, NAME))

print('\n'.join(report))
print('\nBUILD DONE. Extensions bind on the next frame; verify then.')
