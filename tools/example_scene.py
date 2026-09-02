"""
Three synthetic depth cameras looking at one scene from different places.

Camera 1 is the reference. Cameras 2 and 3 see overlapping parts of the same
scene, each in its own space, which is exactly the problem the component
solves. numpy only, no Open3D, so it runs inside TouchDesigner.

This file is the SOURCE of the syntheticScene DAT. tools/td_build_example.py
reads it off disk and installs it into Inputs_1 and Inputs_2, and the demo is
the master saved without TDMCP, so the demo and the local test rig are the same
cameras by construction.

`variant` picks between two separate sets of cameras. Variant 0 is the demo,
variant 1 is a different scene from different places, which is what makes the
Inputs op parameter worth showing: point it at the other base and the whole rig
swaps.
"""

import numpy as np

W, H = 256, 192

# The answers the calibration should find. Camera 2 sits 25 degrees round and
# 40 cm to the side of camera 1; camera 3 is 18 degrees the other way.
POSE = {
	1: (0.0, [0, 1, 0], [0.0, 0.0, 0.0]),
	2: (25.0, [.1, 1, .05], [.40, .03, -.30]),
	3: (-18.0, [0, 1, .1], [-.35, .02, .25]),
}

# Variant 1's cameras sit somewhere else entirely, so a swap is obvious.
POSE_B = {
	1: (0.0, [0, 1, 0], [0.0, 0.0, 0.0]),
	2: (-32.0, [0, 1, .08], [-.55, .04, .20]),
	3: (14.0, [.05, 1, 0], [.30, -.02, .45]),
}


def poses(variant=0):
	return POSE_B if variant else POSE


def rigid(deg, axis, t):
	"""A turn of deg degrees about axis, then a move by t, as a 4x4."""
	axis = np.asarray(axis, dtype=np.float64)
	axis = axis / np.linalg.norm(axis)
	th = np.radians(deg)
	K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
	M = np.eye(4)
	M[:3, :3] = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)
	M[:3, 3] = t
	return M


def scene(seed=0):
	"""A floor, a sphere and a box: enough shape for the matcher to lock onto."""
	rng = np.random.default_rng(seed)
	g = np.linspace(-1.5, 1.5, 120)
	gx, gz = np.meshgrid(g, g)
	floor = np.stack([gx.ravel(), np.zeros(gx.size), gz.ravel()], 1)
	v = rng.normal(size=(6000, 3))
	v /= np.linalg.norm(v, axis=1, keepdims=True)
	b = rng.uniform(-.2, .2, (6000, 3))
	b[np.arange(6000), rng.integers(0, 3, 6000)] = .2 * rng.choice([-1., 1.], 6000)
	return np.vstack([floor, v * .30 + [1.0, .35, .5], b + [-.8, .2, -.6]])


def slice_for(index, points):
	"""Each camera sees an overlapping part of the scene, not all of it."""
	if index == 1:
		return points[points[:, 0] > -1.10]
	if index == 2:
		return points[points[:, 0] < 1.10]
	return points[points[:, 2] < 1.10]


def cloud(index, variant=0):
	"""The cloud camera `index` reports, in that camera's own space."""
	full = scene(seed=variant)
	rng = np.random.default_rng(7 + index + 100 * variant)
	pts = slice_for(index, full)
	pts = pts + rng.normal(scale=.003, size=pts.shape)   # sensor noise
	M = np.linalg.inv(rigid(*poses(variant)[index]))
	return (M[:3, :3] @ pts.T).T + M[:3, 3]


def cloudImage(index, variant=0):
	"""Pack the cloud into an image the way a depth camera TOP does.

	Unused pixels stay exactly zero with alpha 0, which is what the component
	reads as "no point here".
	"""
	pts = cloud(index, variant)
	flat = np.zeros((H * W, 4), dtype=np.float32)
	n = min(len(pts), H * W)
	flat[:n, :3] = pts[:n]
	flat[:n, 3] = 1.0
	return flat.reshape(H, W, 4)
