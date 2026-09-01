"""
Build the TDXDepthCamMerger demo project: three synthetic depth cameras, no
hardware, driven through the shipped .tox.

Run from TD:  exec(open(r'<this file>', encoding='utf-8').read())

The component is loaded from TDXDepthCamMerger.tox rather than built in place,
so the demo exercises the artefact that actually ships.

The clouds are generated procedurally inside a script TOP callback instead of
being stored on the operator. Storing three 256x192 RGBA float arrays would add
roughly 2 MB of pickled numpy to the .toe; generating them costs one cook.

Pythonexe arrives empty because td_build.py clears it in the exported .tox. Set
it if you want to drive a calibration here, then re-run this script to get a
clean demo back before saving.

Saving is a separate script, tools/save_demo.py, because the demo must not ship
the TDMCP tox and destroying TDMCP kills the connection this one arrived on.
"""

import os

REPO = globals().get('REPO') or project.folder.replace(os.sep, '/')
TOX = REPO + '/TDXDepthCamMerger.tox'
NAME = 'TDXDepthCamMerger'
HOST = op('/ProjectName')
W, H = 256, 192

if not os.path.isfile(TOX):
	raise RuntimeError('no component at {}. Export it first.'.format(TOX))

report = []


def note(msg):
	report.append(msg)


# _______________________________________________________ 1. synthetic cameras

for leftover in ('Inputs', NAME, 'calibtest'):
	if HOST.op(leftover):
		HOST.op(leftover).destroy()

inputs = HOST.create(baseCOMP, 'Inputs')
inputs.nodeX, inputs.nodeY = -900, 400
inputs.par.parentshortcut = 'Inputs'

scene = inputs.create(textDAT, 'syntheticScene')
scene.nodeX, scene.nodeY = -400, 300
scene.par.language = 'python'
scene.text = '''"""
Three synthetic depth cameras looking at one scene from different places.

Camera 1 is the reference. Cameras 2 and 3 see overlapping slices of the same
geometry, each expressed in its own frame, so recovering the transforms between
them is exactly the problem the component solves. numpy only, no Open3D: this
runs inside TouchDesigner.
"""

import numpy as np

W, H = 256, 192

# The transforms the calibration should recover. Camera 2 sits 25 degrees round
# and 40 cm to the side of camera 1; camera 3 is 18 degrees the other way.
POSE = {
\t1: (0.0, [0, 1, 0], [0.0, 0.0, 0.0]),
\t2: (25.0, [.1, 1, .05], [.40, .03, -.30]),
\t3: (-18.0, [0, 1, .1], [-.35, .02, .25]),
}


def rigid(deg, axis, t):
\t"""A 4x4 rotation about axis by deg, then translation t."""
\taxis = np.asarray(axis, dtype=np.float64)
\taxis = axis / np.linalg.norm(axis)
\tth = np.radians(deg)
\tK = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
\tM = np.eye(4)
\tM[:3, :3] = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)
\tM[:3, 3] = t
\treturn M


def scene(seed=0):
\t"""A floor, a sphere and a box: enough structure for FPFH to lock onto."""
\trng = np.random.default_rng(seed)
\tg = np.linspace(-1.5, 1.5, 120)
\tgx, gz = np.meshgrid(g, g)
\tfloor = np.stack([gx.ravel(), np.zeros(gx.size), gz.ravel()], 1)
\tv = rng.normal(size=(6000, 3))
\tv /= np.linalg.norm(v, axis=1, keepdims=True)
\tb = rng.uniform(-.2, .2, (6000, 3))
\tb[np.arange(6000), rng.integers(0, 3, 6000)] = .2 * rng.choice([-1., 1.], 6000)
\treturn np.vstack([floor, v * .30 + [1.0, .35, .5], b + [-.8, .2, -.6]])


def slice_for(index, points):
\t"""Each camera sees an overlapping part of the scene, not all of it."""
\tif index == 1:
\t\treturn points[points[:, 0] > -1.10]
\tif index == 2:
\t\treturn points[points[:, 0] < 1.10]
\treturn points[points[:, 2] < 1.10]


def cloud(index):
\t"""The cloud camera `index` reports, in that camera's own frame."""
\tfull = scene()
\trng = np.random.default_rng(7 + index)
\tpts = slice_for(index, full)
\tpts = pts + rng.normal(scale=.003, size=pts.shape)   # sensor noise
\tM = np.linalg.inv(rigid(*POSE[index]))
\treturn (M[:3, :3] @ pts.T).T + M[:3, 3]


def cloudImage(index):
\t"""Pack the cloud into a WxH RGBA float32 image the way a depth TOP does.

\tUnused pixels stay exact zero with alpha 0, which is what the component
\ttreats as invalid.
\t"""
\tpts = cloud(index)
\tflat = np.zeros((H * W, 4), dtype=np.float32)
\tn = min(len(pts), H * W)
\tflat[:n, :3] = pts[:n]
\tflat[:n, 3] = 1.0
\treturn flat.reshape(H, W, 4)
'''

callbacks = inputs.create(textDAT, 'cloud_callbacks')
callbacks.nodeX, callbacks.nodeY = -400, 160
callbacks.par.language = 'python'
callbacks.text = ('# Shared by every cam<n>_points TOP. The camera number comes from the\n'
	'# operator name, so one callback serves all three.\n'
	'\n'
	'def onCook(scriptOp):\n'
	'\tindex = int(\'\'.join(c for c in scriptOp.name if c.isdigit()))\n'
	'\tscriptOp.copyNumpyArray(parent().op(\'syntheticScene\').module.cloudImage(index))\n'
	'\treturn\n')

# A colour per camera so you can see which points came from where once they
# land in one frame. Same resolution as the cloud or the two will not line up.
COLOURS = {1: (0.95, 0.35, 0.25), 2: (0.30, 0.80, 0.45), 3: (0.35, 0.55, 0.95)}

for index in (1, 2, 3):
	y = 200 - 200 * index
	points = inputs.create(scriptTOP, 'cam{}_points'.format(index))
	points.nodeX, points.nodeY = 0, y
	points.par.format = 'rgba32float'
	points.par.callbacks = callbacks
	points.cook(force=True)

	colour = inputs.create(constantTOP, 'cam{}_color'.format(index))
	colour.nodeX, colour.nodeY = 0, y - 90
	colour.par.resolutionw, colour.par.resolutionh = W, H
	r, g, b = COLOURS[index]
	colour.par.colorr, colour.par.colorg, colour.par.colorb = r, g, b
	colour.par.alpha = 1

note('built 3 synthetic cameras at {}x{}'.format(W, H))

# _______________________________________________________ 2. the component

comp = HOST.loadTox(TOX)
comp.nodeX, comp.nodeY = 0, 400
note('loaded {} from the shipped .tox: {} descendants, {} errors'.format(
	comp.name, len(comp.findChildren(maxDepth=99)), len(comp.errors(recurse=True))))

rows = [['name', 'pointcloud', 'color', 'mask']]
for index in (1, 2, 3):
	rows.append(['cam{}'.format(index),
		'/ProjectName/Inputs/cam{}_points'.format(index),
		'/ProjectName/Inputs/cam{}_color'.format(index),
		''])
table = comp.op('customSources')
table.clear()
for row in rows:
	table.appendRow(row)

comp.par.Devicetype = 'custom'
note('customSources filled with 3 rows, Devicetype = custom')

# _______________________________________________________ 3. instructions

readme = HOST.create(textDAT, 'START_HERE')
readme.nodeX, readme.nodeY = -900, 700
readme.par.language = 'python'
readme.text = '''"""
TDXDepthCamMerger demo. Three synthetic depth cameras, no hardware needed.

Inputs/ generates three overlapping point clouds of one scene, each in its own
camera frame, coloured red, green and blue. They start misaligned. Calibrating
brings them into a single coordinate space.

To run it:

  1. Select TDXDepthCamMerger. On the Setup page set Python exe to a
     python.exe that has open3d installed, then pulse Check worker. Worker
     status should report the versions. See the README for the install.

  2. Pulse Gather devices. Three devices appear, one per customSources row.

  3. Calibrate page: set Specify pair to 1 and 2, then pulse Calibrate.
     Watch the green cloud swing onto the red one.

  4. Set Specify pair to 2 and 3 and pulse Calibrate again. Blue joins them,
     composed through camera 2 into camera 1's frame.

The recovered transforms should match the ones in Inputs/syntheticScene POSE:
25 degrees and 40 cm for camera 2, -18 degrees the other way for camera 3.
Last fitness and Last RMSE report how well it did.
"""
'''
note('added START_HERE')

print('\n'.join(report))
print('\nDEMO BUILT. Set Pythonexe to verify, and clear it again before saving.')
