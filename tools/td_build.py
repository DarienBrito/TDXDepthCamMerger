"""
Build TDXDepthCameraMerger 0.2.0 inside a live TouchDesigner 2025.33070.

Run from TD (MCP execute_code):

    exec(open(r'<this file>', encoding='utf-8').read())

Idempotent: destroys and rebuilds the component each time, so it can be re-run
after a crash without hand work.

Why a fresh container rather than loading the .tox directly: the 2023.12370
container COMP's own saved state crashes TD 2025.33070 when it cooks. Its 145
children are all fine, so we make a new container and copyOPs them in.
"""

import json
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
NAME = 'TDXDepthCameraMerger'
SHORTCUT = 'TDXMerger'
OLD_SHORTCUT = 'TDAzureMerger'
PYEXE = os.environ.get('TDX_PYTHON_EXE') or 'D:/anaconda3/envs/td/python.exe'

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

with open(BUILD + '/custom_pars.json', encoding='utf-8') as h:
	saved = json.load(h)

DROP = {'Asintermediary', 'Currentpair', 'Numberofpairs',
	'Specifypairx', 'Specifypairy', 'Useplayerforcalibration', 'Gatherkinects'}
APPEND = {'OP': 'appendOP', 'Pulse': 'appendPulse', 'Int': 'appendInt',
	'Str': 'appendStr', 'Toggle': 'appendToggle', 'Menu': 'appendMenu'}

pages = {n: comp.appendCustomPage(n) for n in ('Configuration', 'Calibration', 'About')}

for e in sorted([s for s in saved if s['name'] not in DROP], key=lambda x: x['order']):
	page = pages[e['page']]
	par = getattr(page, APPEND[e['style']])(e['name'], label=e['label'])[0]
	if e['style'] == 'Menu':
		par.menuNames, par.menuLabels = e['menuNames'], e['menuLabels']
	if e['style'] not in ('Pulse', 'OP') and e.get('val') is not None:
		par.val = e['val']
	if e.get('readOnly'):
		par.readOnly = True

cfg, cal, abt = pages['Configuration'], pages['Calibration'], pages['About']

cfg.appendMenu('Devicetype', label='Device type')[0]
comp.par.Devicetype.menuNames = ['kinectazure', 'orbbec', 'custom']
comp.par.Devicetype.menuLabels = ['Kinect Azure', 'Orbbec', 'Custom TOPs']
cfg.appendPulse('Gatherdevices', label='Gather devices')
cfg.appendFile('Pythonexe', label='Python exe')[0].val = PYEXE
cfg.appendPulse('Checkworker', label='Check worker')
cfg.appendStr('Open3dstatus', label='Worker status')[0].readOnly = True

cal.appendInt('Referencedevice', label='Reference device')[0].val = 1
cal.appendInt('Specifypair', label='Specify pair', size=2)
comp.par.Specifypair1.val, comp.par.Specifypair2.val = 1, 2
cal.appendToggle('Usemaskforcalibration', label='Use mask for calibration')
cal.appendFloat('Voxelsize', label='Voxel size (m)')[0].val = 0.05
cal.appendFloat('Refinevoxel', label='Refine voxel (m)')[0].val = 0.01
cal.appendFloat('Maxrange', label='Max range (m)')[0].val = 0.0
cal.appendToggle('Usecoloricp', label='Use coloured ICP')
cal.appendInt('Seed', label='RANSAC seed')[0].val = -1
cal.appendDAT('Presetmatrixdat', label='Preset matrix DAT')
cal.appendPulse('Rebuildchain', label='Rebuild chain')
cal.appendPulse('Resetcalibration', label='Reset calibration')
for n, l in (('Lastfitness', 'Last fitness'), ('Lastrmse', 'Last RMSE'),
		('Laststatus', 'Last status')):
	cal.appendStr(n, label=l)[0].readOnly = True
cal.appendInt('Lastcorrespondences', label='Last correspondences')[0].readOnly = True

comp.par.Version = '0.2.0'
comp.par.Mode.menuNames = ['globalThenIcp', 'globalRegistration', 'table']
comp.par.Mode.menuLabels = ['Global + ICP refine', 'Global registration only',
	'From table DAT']
# Chained is the default: calibrating from scratch wants both stages, and doing
# them in one worker run costs one process start instead of two.
comp.par.Mode.val = 'globalThenIcp'
note('custom parameters: {}'.format(len(comp.customPars)))


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


installDat('extTDAzureMerger', SRC + '/extTDAzureMerger.py', -900, 200)
installDat('extUtilities', SRC + '/extUtilities.py', -900, 80)
installDat('workerSource', SRC + '/worker.py', -900, -40)
note('installed extension, utilities and worker source from {}'.format(SRC))
note('python exe: {}'.format(PYEXE))

comp.op('parexec1').text = '''# Routes root parameter pulses to the extensions.

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
	return
'''
comp.op('parexec1').par.ops = comp
comp.op('parexec1').par.pars = '*'


# ____________________________________________________ 6. extensions

# The Extension Object parameter holds python CODE AS A CONSTANT STRING.
# Setting it as an expression silently fails to bind.
comp.par.ext0object.mode = ParMode.CONSTANT
comp.par.ext0object.val = "op('./extTDAzureMerger').module.extTDAzureMerger(me)"
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

# The Readme was a containerCOMP wrapping a webrenderTOP left active, so every
# instance started a CEF browser process on load. actions/parexec1 already
# opened Support in the real browser; the Readme now goes the same way.
actions = comp.op('actions')
if actions.op('Readme'):
	actions.op('Readme').destroy()
	stripped.append('actions/Readme')
# 'Help' is a 0.0.3 leftover: the pattern matches no parameter on this component.
actions.op('parexec1').par.pars = 'Readme Support'
actions.op('parexec1').text = """import webbrowser

README = 'https://github.com/DarienBrito/TDAzureMerger#readme'
SUPPORT = 'https://darienbrito.com/support/'


def onPulse(par):
	if par.name == 'Readme':
		webbrowser.open(README)
	elif par.name == 'Support':
		webbrowser.open(SUPPORT)
	return
"""

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


comp.cook(force=True)
errs = [(o.path.split(NAME + '/')[-1], o.errors().replace('\n', ' ')[:100])
	for o in comp.findChildren(maxDepth=99) if o.errors()]
note('errors after build: {}'.format(len(errs)))
for p, e in errs:
	note('    E {} -> {}'.format(p, e))

print('\n'.join(report))
print('\nBUILD DONE. Extensions bind on the next frame; verify then.')
