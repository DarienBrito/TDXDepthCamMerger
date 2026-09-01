"""
End to end test of Calibrate / Refine / RebuildChain INSIDE TouchDesigner,
with no camera attached.

Builds a harness that presents the same interface the real component does
(Device<n> COMPs each holding a null_sourcePointcloud TOP and a transformMatrix
table), fills the TOPs with synthetic clouds related by a known 4x4, then drives
the real extension and checks the recovered matrix.

Run from TD:  exec(open(r'<this file>', encoding='utf-8').read())
"""

import os

import numpy as np

PYEXE = os.environ.get('TDX_PYTHON_EXE') or 'D:/anaconda3/envs/td/python.exe'
W, H = 256, 192

HOST = op('/ProjectName')
results = []


def check(name, ok, detail=''):
	results.append((name, bool(ok), detail))
	print('  [{}] {}{}'.format('PASS' if ok else 'FAIL', name, ('  -> ' + detail) if detail else ''))


def rigid(deg, axis, t):
	axis = np.asarray(axis, dtype=np.float64)
	axis = axis / np.linalg.norm(axis)
	th = np.radians(deg)
	K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
	M = np.eye(4)
	M[:3, :3] = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)
	M[:3, 3] = t
	return M


def poseError(a, b):
	d = np.linalg.inv(a) @ b
	return (np.linalg.norm(d[:3, 3]),
		np.degrees(np.arccos(np.clip((np.trace(d[:3, :3]) - 1) / 2, -1, 1))))


def scene(seed=0):
	rng = np.random.default_rng(seed)
	g = np.linspace(-1.5, 1.5, 120)
	gx, gz = np.meshgrid(g, g)
	floor = np.stack([gx.ravel(), np.zeros(gx.size), gz.ravel()], 1)
	v = rng.normal(size=(6000, 3))
	v /= np.linalg.norm(v, axis=1, keepdims=True)
	b = rng.uniform(-.2, .2, (6000, 3))
	b[np.arange(6000), rng.integers(0, 3, 6000)] = .2 * rng.choice([-1., 1.], 6000)
	return np.vstack([floor, v * .30 + [1.0, .35, .5], b + [-.8, .2, -.6]])


def asImage(points):
	"""Pack an Nx3 cloud into a WxH RGBA float32 image, padded with invalid zeros."""
	flat = np.zeros((H * W, 4), dtype=np.float32)
	n = min(len(points), H * W)
	flat[:n, :3] = points[:n]
	flat[:n, 3] = 1.0
	return flat.reshape(H, W, 4)


# ____________________________________________________________ build harness

if HOST.op('calibtest'):
	HOST.op('calibtest').destroy()
rig = HOST.create(containerCOMP, 'calibtest')
rig.nodeX, rig.nodeY = 900, 400
rig.par.parentshortcut = 'CalibTest'

src = op('/ProjectName/TDXDepthCamMerger')
for name in ('extTDXDepthCamMerger', 'workerSource'):
	d = rig.create(textDAT, name)
	d.text = src.op(name).text
cal = rig.create(tableDAT, 'calibrationData')
cal.clear()

page = rig.appendCustomPage('Calibration')
page.appendFile('Pythonexe')[0].val = PYEXE
page.appendFloat('Voxelsize')[0].val = 0.05
page.appendFloat('Refinevoxel')[0].val = 0.01
page.appendFloat('Maxrange')[0].val = 0.0
page.appendInt('Seed')[0].val = 3
page.appendToggle('Usecoloricp')
page.appendInt('Referencedevice')[0].val = 1
page.appendInt('Numberofdevices')[0].val = 3
page.appendStr('Lastfitness')
page.appendStr('Lastrmse')
page.appendStr('Laststatus')
page.appendInt('Lastcorrespondences')
page.appendStr('Open3dstatus')
page.appendDAT('Presetmatrixdat')

TRUE12 = rigid(25, [.1, 1, .05], [.40, .03, -.30])
TRUE23 = rigid(-18, [0, 1, .1], [-.35, .02, .25])

full = scene()
rng = np.random.default_rng(7)
c1 = full[full[:, 0] > -1.10] + rng.normal(scale=.003, size=full[full[:, 0] > -1.10].shape)
c2 = full[full[:, 0] < 1.10] + rng.normal(scale=.003, size=full[full[:, 0] < 1.10].shape)
c3 = full[full[:, 2] < 1.10] + rng.normal(scale=.003, size=full[full[:, 2] < 1.10].shape)

# Device1 sees the scene already in the reference frame; Device2's cloud is the
# same scene expressed in its own frame, i.e. TRUE12 maps device2 -> device1.
clouds = {
	1: (TRUE12[:3, :3] @ c1.T).T + TRUE12[:3, 3],
	2: c2,
	3: (np.linalg.inv(TRUE23)[:3, :3] @ c3.T).T + np.linalg.inv(TRUE23)[:3, 3],
}

for index, points in clouds.items():
	dev = rig.create(baseCOMP, 'Device{}'.format(index))
	dev.nodeX, dev.nodeY = 0, -200 * index
	top = dev.create(scriptTOP, 'null_sourcePointcloud')
	top.par.format = 'rgba32float'
	cb = dev.create(textDAT, 'null_sourcePointcloud_callbacks')
	cb.text = ('def onCook(scriptOp):\n'
		'\tarr = scriptOp.fetch("cloud", None)\n'
		'\tif arr is not None:\n'
		'\t\tscriptOp.copyNumpyArray(arr)\n'
		'\treturn\n')
	top.par.callbacks = cb
	top.store('cloud', asImage(points))
	top.cook(force=True)
	t = dev.create(tableDAT, 'transformMatrix')
	t.clear()

rig.par.ext0object.mode = ParMode.CONSTANT
rig.par.ext0object.val = "op('./extTDXDepthCamMerger').module.extTDXDepthCamMerger(me)"
rig.par.ext0promote = True
rig.par.reinitextensions.pulse()

# Stash the ground truth so a follow up script can grade a run without rebuilding
# the scene or duplicating the maths. td_test_devicesources.py reads these.
rig.store('TRUE12', TRUE12)
rig.store('TRUE23', TRUE23)

print('harness built: devices', sorted(o.name for o in rig.children if o.name.startswith('Device')))
print('TRUE12 and TRUE23 seeded and stored on the rig; drive Calibrate next frame')
