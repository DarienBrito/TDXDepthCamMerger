"""
Build the local test rig in the OPEN project: two bases of synthetic cameras and
the component pointed at one of them.

Run from TD:  exec(open(r'<this file>', encoding='utf-8').read())

This is the rig to develop against, not the demo: it leaves the component where
td_build.py put it, sets Python exe to the local one, and adds a SECOND camera
base so the Inputs op swap can be tried by hand.

    Inputs_1  variant 0, the same three cameras the demo ships
    Inputs_2  variant 1, a different scene from different places

Both hold plain names in customSources, so switching the component's Inputs op
between them swaps the whole rig. The scene comes from tools/example_scene.py,
which the demo builder reads too, so the two cannot drift.

The bases are DESTROYED and rebuilt every run, so where their nodes sit comes
from the `project` section of tools/build/layout.json, captured from the master
by tools/td_capture_layout.py. Hand arrangement out here used to be wiped by
every run. Move things, then re-capture.

Order: tools/td_build.py first, then this.
"""

import json
import os

REPO = globals().get('REPO') or project.folder.replace(os.sep, '/')
SCENE = REPO + '/tools/example_scene.py'
NAME = 'TDXDepthCamMerger'
HOST = op('/ProjectName')
PYEXE = os.environ.get('TDX_PYTHON_EXE') or 'D:/anaconda3/envs/td/python.exe'
W, H = 256, 192

# One colour per camera per base, so you can tell which cloud came from where,
# and a swap between the two bases is obvious.
COLOURS = {
	0: {1: (0.95, 0.35, 0.25), 2: (0.30, 0.80, 0.45), 3: (0.35, 0.55, 0.95)},
	1: {1: (0.95, 0.80, 0.25), 2: (0.75, 0.35, 0.85), 3: (0.25, 0.85, 0.85)},
}

CALLBACK = (
	"# Shared by every cam<n>_points TOP. The camera number comes from the\n"
	"# operator name and the scene from the base's Variant parameter, so one\n"
	"# callback serves every TOP in every base.\n"
	"\n"
	"def onCook(scriptOp):\n"
	"\tindex = int(''.join(c for c in scriptOp.name if c.isdigit()))\n"
	"\tvariant = int(parent().par.Variant)\n"
	"\tscene = parent().op('syntheticScene').module\n"
	"\tscriptOp.copyNumpyArray(scene.cloudImage(index, variant))\n"
	"\treturn\n")

if not os.path.isfile(SCENE):
	raise RuntimeError('no scene source at {}'.format(SCENE))

LAYOUT = REPO + '/tools/build/layout.json'
PROJECT = {}
if os.path.isfile(LAYOUT):
	with open(LAYOUT, encoding='utf-8') as handle:
		PROJECT = json.load(handle).get('project', {})


def place(node, path, x, y):
	"""
	Put a node where the master has it. The literals are only a fallback for a
	repo with no capture yet; whenever layout.json names the path it wins, so a
	rebuild cannot move anything the user arranged.
	"""
	node.nodeX, node.nodeY = PROJECT.get(path, (x, y))

comp = HOST.op(NAME)
if comp is None:
	raise RuntimeError('no {} in the project. Run tools/td_build.py first.'.format(NAME))

report = []


def note(msg):
	report.append(msg)


def buildBase(name, variant, x, y):
	"""One base of three synthetic depth cameras."""
	if HOST.op(name):
		HOST.op(name).destroy()
	base = HOST.create(baseCOMP, name)
	place(base, name, x, y)
	base.par.parentshortcut = name

	page = base.appendCustomPage('Scene')
	page.appendInt('Variant')[0].val = variant

	scene = base.create(textDAT, 'syntheticScene')
	place(scene, name + '/syntheticScene', -400, 300)
	scene.par.language = 'python'
	scene.text = open(SCENE, encoding='utf-8').read()

	callbacks = base.create(textDAT, 'cloud_callbacks')
	place(callbacks, name + '/cloud_callbacks', -400, 160)
	callbacks.par.language = 'python'
	callbacks.text = CALLBACK

	for index in (1, 2, 3):
		row = 200 - 200 * index
		points = base.create(scriptTOP, 'cam{}_points'.format(index))
		place(points, '{}/{}'.format(name, points.name), 0, row)
		points.par.format = 'rgba32float'
		points.par.callbacks = callbacks
		points.cook(force=True)

		# Creating a scriptTOP also makes a companion <name>_callbacks DAT.
		# Nothing here uses it, since every camera shares cloud_callbacks above,
		# but it is a real node in the network and it has to be placed or the
		# rebuild drops it back on TD's default offset.
		spare = base.op(points.name + '_callbacks')
		if spare is not None:
			place(spare, '{}/{}'.format(name, spare.name), 0, row - 120)

		colour = base.create(constantTOP, 'cam{}_color'.format(index))
		place(colour, '{}/{}'.format(name, colour.name), 0, row - 90)
		colour.par.resolutionw, colour.par.resolutionh = W, H
		r, g, b = COLOURS[variant][index]
		colour.par.colorr, colour.par.colorg, colour.par.colorb = r, g, b
		colour.par.alpha = 1

	note('{}: variant {}, 3 cameras at {}x{}'.format(name, variant, W, H))
	return base


inputs = buildBase('Inputs_1', 0, -300, 400)
buildBase('Inputs_2', 1, -300, 200)

# Plain names, not full paths: that is what makes Inputs op the switch.
table = comp.op('customSources')
table.clear()
table.appendRow(['name', 'pointcloud', 'color', 'mask'])
for index in (1, 2, 3):
	table.appendRow(['cam{}'.format(index),
		'cam{}_points'.format(index), 'cam{}_color'.format(index), ''])

comp.par.Pythonexe = PYEXE
comp.par.Inputsop = inputs.path
comp.par.Devicetype = 'custom'
comp.par.Referencedevice = 1
comp.par.Specifypair1, comp.par.Specifypair2 = 1, 2
comp.GatherDevices(warn=False)
note('component: Devicetype custom, Inputs op {}, {} devices'.format(
	inputs.path, int(comp.par.Numberofdevices)))


print('\n'.join(report))
print('\nEXAMPLE RIG BUILT. Calibrate pair 1-2, then 2-3.')
