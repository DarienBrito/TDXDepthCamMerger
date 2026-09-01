"""
TDXDepthCamMerger registration worker.

Runs OUTSIDE TouchDesigner, in any python 3.11+ that has open3d installed.
Importing open3d inside TouchDesigner hard-crashes the process (duplicate native
runtimes: open3d ships its own OpenMP/TBB alongside TouchDesigner's), so the
registration is done here and only a 4x4 matrix travels back.

    python worker.py --probe          report interpreter and open3d versions
    python worker.py job.json         run a registration job

The job file is JSON:

    {
      "mode":         "global" | "icp" | "globalThenIcp",
      "target":       "<path>.npy",     Nx3 float64, the reference cloud
      "source":       "<path>.npy",     Nx3 float64, the cloud being moved
      "targetColors": "<path>.npy"|null Nx3 float64 0..1, coloured ICP only
      "sourceColors": "<path>.npy"|null
      "voxel":        0.05,             metres, coarse/RANSAC resolution
      "refineVoxel":  0.01,             metres, ICP resolution (0 = full res)
      "maxRange":     0.0,              metres, 0 = no distance crop
      "seed":         -1,               >=0 pins OpenMP to 1 thread and seeds
                                        Open3D, which makes RANSAC reproducible
                                        at the cost of the parallel speedup
      "colored":      false,
      "init":         [16 floats]|null  row-major seed matrix for ICP
    }

The result is JSON on stdout and, if "result" is given, written to that path:

    {"ok": true, "matrix": [16 floats, row-major], "fitness": .., "rmse": ..,
     "correspondences": .., "stage": "global"|"icp"|"coloredIcp"|"globalThenIcp"}

"globalThenIcp" runs RANSAC and then seeds ICP with its result in one process,
which is what a calibration from scratch actually wants. Its reply carries the
final ICP numbers plus a "global" sub-dict holding the first stage's, so both
are visible. If the global stage grades FAIL the ICP is skipped and the global
result comes back as is: seeded from the wrong basin, ICP only polishes a wrong
alignment into a confident-looking one.
    {"ok": false, "error": "...", "traceback": "..."}

Convention: the returned matrix M satisfies x_target = M @ x_source, with
homogeneous column vectors.
"""

import json
import os
import sys
import traceback

import numpy as np

EPS = 1e-6

FITNESS_FAIL = 0.05
FITNESS_WARN = 0.25
ICP_FITNESS_WARN = 0.4
CORRESPONDENCES_FAIL = 100
RMSE_WARN_FACTOR = 0.6


# ____________________________________________________________ cloud building


def validMask(points, maxRange=0.0):
	"""
	True where a point is usable. Depth cameras write exactly (0,0,0) for pixels
	with no return; left in, those land as a dense phantom cluster at the origin
	of both clouds and drag the registration towards it.
	"""
	mask = np.isfinite(points).all(axis=1) & (np.abs(points) > EPS).any(axis=1)
	if maxRange and maxRange > 0:
		mask &= np.einsum('ij,ij->i', points, points) < float(maxRange) ** 2
	return mask


def loadPoints(path, maxRange=0.0):
	points = np.load(path)
	points = points.reshape(-1, points.shape[-1])[:, :3].astype(np.float64)
	return points, validMask(points, maxRange)


def makeCloud(o3d, points, colors=None):
	cloud = o3d.geometry.PointCloud()
	cloud.points = o3d.utility.Vector3dVector(points)
	if colors is not None and len(colors):
		cloud.colors = o3d.utility.Vector3dVector(colors)
	return cloud


def downsample(cloud, voxel):
	"""A voxel of 0 means leave it alone, the full resolution escape hatch."""
	return cloud.voxel_down_sample(voxel) if voxel and voxel > 0 else cloud


def estimateNormals(o3d, cloud, radius, maxNn=30):
	"""
	estimate_normals leaves each normal's sign arbitrary. FPFH and point to plane
	ICP both care. Every cloud arrives in its own camera's frame with the camera
	at the origin, so orienting towards the origin is exactly right.
	"""
	cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=maxNn))
	cloud.orient_normals_towards_camera_location(np.zeros(3))
	return cloud


def featureCloud(o3d, cloud, voxel):
	down = downsample(cloud, voxel)
	estimateNormals(o3d, down, radius=voxel * 2)
	feature = o3d.pipelines.registration.compute_fpfh_feature(
		down, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100))
	return down, feature


# ______________________________________________________________ registration


def globalRegistration(o3d, sourceDown, targetDown, sourceFpfh, targetFpfh, voxel, seed=-1):
	"""
	RANSAC over FPFH correspondences: the initial guess. Each iteration draws
	ransac_n points from the source and matches them by nearest neighbour in the
	33 dimensional feature space; the checkers prune bad matches early.

	Fast Point Feature Histograms (FPFH) for 3D registration, ICRA, 2009.
	"""
	if seed is not None and int(seed) >= 0:
		o3d.utility.random.seed(int(seed))
	reg = o3d.pipelines.registration
	distance = voxel * 1.5
	return reg.registration_ransac_based_on_feature_matching(
		sourceDown, targetDown, sourceFpfh, targetFpfh,
		mutual_filter=True,
		max_correspondence_distance=distance,
		estimation_method=reg.TransformationEstimationPointToPoint(False),
		ransac_n=3,
		checkers=[
			reg.CorrespondenceCheckerBasedOnEdgeLength(0.9),
			reg.CorrespondenceCheckerBasedOnDistance(distance),
		],
		# Open3D's own default. The confidence test terminates long before the
		# iteration cap, so the original 2,000,000 bought nothing.
		criteria=reg.RANSACConvergenceCriteria(100000, 0.999))


def icpRefine(o3d, source, target, distance, matrix, colored=False, maxIterations=50):
	"""
	ICP seeded with matrix. Point to plane by default: depth camera clouds are
	locally planar, so it converges faster and tighter than point to point, and
	the normals are already paid for. target must carry normals; coloured ICP
	additionally needs colours on both clouds.
	"""
	reg = o3d.pipelines.registration
	criteria = reg.ICPConvergenceCriteria(max_iteration=int(maxIterations))
	if colored:
		return reg.registration_colored_icp(
			source, target, distance, matrix,
			reg.TransformationEstimationForColoredICP(), criteria)
	return reg.registration_icp(
		source, target, distance, matrix,
		reg.TransformationEstimationPointToPlane(), criteria)


def grade(result, distance, stage):
	"""Turn a RegistrationResult into numbers plus a verdict."""
	fitness = float(result.fitness)
	rmse = float(result.inlier_rmse)
	count = len(result.correspondence_set)

	if stage == 'global':
		poor = fitness < FITNESS_WARN
	else:
		poor = fitness < ICP_FITNESS_WARN or rmse > RMSE_WARN_FACTOR * distance

	if fitness < FITNESS_FAIL or count < CORRESPONDENCES_FAIL:
		status = 'FAIL'
	elif poor:
		status = 'WARN'
	else:
		status = 'OK'

	return {'stage': stage, 'fitness': fitness, 'rmse': rmse,
		'correspondences': count, 'distance': float(distance), 'status': status}


# _____________________________________________________________________ entry


def reply(matrix, metrics, targetPts, sourcePts):
	out = {'ok': True, 'matrix': [float(v) for v in np.asarray(matrix).reshape(-1)]}
	out.update(metrics)
	out['targetPoints'] = int(len(targetPts))
	out['sourcePoints'] = int(len(sourcePts))
	return out


def run(job):
	# Open3D parallelises RANSAC over OpenMP and the worker threads race, so
	# utility.random.seed on its own does not reproduce a run: measured 4 distinct
	# results in 6 identical jobs on 0.19.0. Pinning to one thread does reproduce
	# it, byte for byte. Only done when a seed was actually asked for, because it
	# costs the parallel speedup. Must precede the import: OpenMP reads this when
	# the extension module loads.
	seed = int(job.get('seed', -1))
	if seed >= 0:
		os.environ['OMP_NUM_THREADS'] = '1'

	import open3d as o3d

	voxel = float(job.get('voxel', 0.05))
	refineVoxel = float(job.get('refineVoxel', 0.01))
	maxRange = float(job.get('maxRange', 0.0))
	colored = bool(job.get('colored', False))

	targetPts, targetMask = loadPoints(job['target'], maxRange)
	sourcePts, sourceMask = loadPoints(job['source'], maxRange)
	targetPts = targetPts[targetMask]
	sourcePts = sourcePts[sourceMask]
	if not len(targetPts) or not len(sourcePts):
		raise ValueError('a cloud has no valid points after filtering '
			'(target {}, source {})'.format(len(targetPts), len(sourcePts)))

	targetCol = sourceCol = None
	if colored:
		if not job.get('targetColors') or not job.get('sourceColors'):
			raise ValueError('coloured ICP needs colours for both clouds')
		targetCol = np.load(job['targetColors']).reshape(-1, 3)[targetMask]
		sourceCol = np.load(job['sourceColors']).reshape(-1, 3)[sourceMask]

	target = makeCloud(o3d, targetPts, targetCol)
	source = makeCloud(o3d, sourcePts, sourceCol)

	mode = job.get('mode', 'global')
	globalMetrics = None

	if mode in ('global', 'globalThenIcp'):
		targetDown, targetFpfh = featureCloud(o3d, target, voxel)
		sourceDown, sourceFpfh = featureCloud(o3d, source, voxel)
		result = globalRegistration(o3d, sourceDown, targetDown,
			sourceFpfh, targetFpfh, voxel, seed)
		metrics = grade(result, voxel * 1.5, 'global')
		init = np.array(result.transformation, dtype=np.float64)
		# Refining a failed global means polishing the wrong basin, which turns a
		# visibly bad answer into a confident looking one. Hand the failure back.
		if mode == 'global' or metrics['status'] == 'FAIL':
			return reply(init, metrics, targetPts, sourcePts)
		globalMetrics = metrics
	else:
		init = job.get('init')
		init = np.eye(4) if not init else np.array(init, dtype=np.float64).reshape(4, 4)

	targetFine = downsample(target, refineVoxel)
	sourceFine = downsample(source, refineVoxel)
	estimateNormals(o3d, targetFine, radius=(refineVoxel or voxel) * 2)
	distance = (refineVoxel or voxel) * 2.5
	result = icpRefine(o3d, sourceFine, targetFine, distance, init, colored)
	metrics = grade(result, distance, 'coloredIcp' if colored else 'icp')
	if globalMetrics is not None:
		metrics['stage'] = 'globalThenIcp'
		metrics['global'] = globalMetrics

	# result.transformation is read only and Fortran ordered; copy it.
	matrix = np.array(result.transformation, dtype=np.float64)
	return reply(matrix, metrics, targetPts, sourcePts)


def probe():
	info = {'ok': True, 'python': sys.version.split()[0], 'executable': sys.executable}
	try:
		info['numpy'] = np.__version__
	except Exception as err:
		info['numpy'] = 'error: {}'.format(err)
	try:
		import open3d as o3d
		info['open3d'] = o3d.__version__
	except Exception as err:
		info['ok'] = False
		info['open3d'] = None
		info['error'] = '{}: {}'.format(type(err).__name__, err)
	return info


def main():
	args = sys.argv[1:]
	if not args:
		print(json.dumps({'ok': False, 'error': 'no job file given'}))
		return 2
	if args[0] == '--probe':
		print(json.dumps(probe()))
		return 0

	resultPath = None
	try:
		with open(args[0], encoding='utf-8') as handle:
			job = json.load(handle)
		resultPath = job.get('result')
		out = run(job)
	except Exception as err:
		out = {'ok': False, 'error': '{}: {}'.format(type(err).__name__, err),
			'traceback': traceback.format_exc()}

	payload = json.dumps(out)
	if resultPath:
		with open(resultPath, 'w', encoding='utf-8') as handle:
			handle.write(payload)
	print(payload)
	return 0 if out.get('ok') else 1


# __name__ is '__main__' for a TouchDesigner DAT module too, so the plain guard
# is not enough: reading workerSource.module inside TD ran main(), which picked
# up TD's own sys.argv and wedged the process on sys.exit(). Check that we were
# actually launched as this script.
if __name__ == '__main__' and os.path.basename(sys.argv[0] or '').startswith('worker'):
	sys.exit(main())
