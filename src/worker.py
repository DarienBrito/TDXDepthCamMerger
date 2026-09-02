"""
TDXDepthCamMerger registration worker.

Runs OUTSIDE TouchDesigner, in any python 3.11+ that has open3d installed.
Importing open3d inside TouchDesigner crashes it, so the maths happens here and
only a 4x4 matrix travels back.

    python worker.py --probe          report python and open3d versions
    python worker.py job.json         run a registration job

The job file is JSON:

    {
      "mode":         "global" | "icp" | "globalThenIcp",
      "target":       "<path>.npy",     Nx3 float64, the cloud that stays put
      "source":       "<path>.npy",     Nx3 float64, the cloud being moved
      "targetColors": "<path>.npy"|null Nx3 float64 0..1, coloured ICP only
      "sourceColors": "<path>.npy"|null
      "voxel":        0.05,             metres, detail of the rough match
      "refineVoxel":  0.01,             metres, detail of the refine (0 = full)
      "maxRange":     0.0,              metres, 0 = keep everything
      "seed":         -1,               >=0 makes the run repeatable, at the
                                        cost of running on one thread
      "consensusRuns": 4,               how many times the rough match runs. How
                                        far apart the answers land is the only
                                        reliable warning that one is wrong. Set
                                        to 1 when a seed is given. 1 turns the
                                        check off
      "colored":      false,
      "init":         [16 floats]|null  starting matrix for ICP, row by row
    }

The result is JSON printed out and, if "result" is given, written to that path:

    {"ok": true, "matrix": [16 floats, row by row], "fitness": .., "rmse": ..,
     "correspondences": .., "stage": "global"|"icp"|"coloredIcp"|"globalThenIcp",
     "consensus": .., "consensusLimit": .., "agreed": true|false,
     "overlap": ..}

"consensus" is there whenever the rough match ran more than once. Read it before
"fitness": fitness says the solver settled, consensus says the answer came out
the same each time, and only the second tracks whether it is right.

"overlap" is how much of the target camera's view the source camera also sees,
once the matrix is applied. It is what decides whether a pair can be registered
at all, so it is worth reading even though it is measured AFTER the fact: it
describes the answer just found, not the truth, and a wrong pose reports the
overlap of that wrong pose.

"globalThenIcp" does the rough match and the refine in one run, which is what
calibrating from scratch wants. Its reply carries the refined numbers plus a
"global" entry holding the rough ones, so both are visible. If the rough match
fails the refine is skipped: polishing a wrong answer only makes it look
confident.

    {"ok": false, "error": "...", "traceback": "..."}

The matrix M that comes back satisfies x_target = M @ x_source.
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

# Two rough matches further apart than voxel * this are not agreeing. Scaled by
# voxel because the gap is a distance and voxel is the only scale on hand.
# Measured over 60 pairs (15 wrong, 44 right), as caught / doubted:
#
#     factor  4    100% / 82%      factor 12   80% / 16%
#     factor  6     93% / 48%      factor 15   67% / 14%
#     factor  8     93% / 27%      factor 20   53% /  9%
#
# 8 is chosen over the quieter settings on purpose. A "false alarm" here is a
# right answer the matcher reached shakily, so the next press of Calibrate may
# well land wrong: saying so is an early warning, not noise. That only holds
# while the warning is cheap, so it writes the status line and nothing more.
CONSENSUS_WARN_FACTOR = 8.0

# Overlap is measured the same way tests/synth.py measures it, at the same
# radius, so the figure the accuracy sweep established carries over: at or above
# 0.55 shared surface 86 to 100% of pairs recovered the pose, below it 27%, and
# the failures are the wrong rotational basin rather than near misses. Change
# the radius and that number stops meaning anything.
OVERLAP_RADIUS = 0.05
OVERLAP_SAMPLES = 4000


# ____________________________________________________________ cloud building
#
# Every function that needs open3d takes the module as its first argument. The
# import is deferred into run(), because OMP_NUM_THREADS has to be set before
# it (see run()); passing the module along keeps that deferral visible instead
# of hiding it in a module global.


def validMask(points, maxRange=0.0):
	"""
	True where a point can be used. Depth cameras write exactly (0,0,0) for
	pixels they got nothing back from; left in, those pile up at the origin of
	both clouds and drag the match towards it.
	"""
	mask = np.isfinite(points).all(axis=1) & (np.abs(points) > EPS).any(axis=1)
	if maxRange and maxRange > 0:
		mask &= np.einsum('ij,ij->i', points, points) < float(maxRange) ** 2
	return mask


def loadPoints(path, maxRange=0.0):
	points = np.load(path)
	points = points.reshape(-1, points.shape[-1])[:, :3].astype(np.float64)
	return points, validMask(points, maxRange)


def loadColors(path, mask, label):
	"""
	One colour per point that survived filtering.

	The colour image has to carry one pixel per point. Numpy's own complaint
	about a mismatch says nothing about what to do, so say it here.
	"""
	colors = np.load(path).reshape(-1, 3)
	if len(colors) != len(mask):
		raise ValueError(
			f'the {label} colour image has {len(colors)} pixels but its point '
			f'cloud has {len(mask)}. Coloured ICP needs them aligned pixel for '
			'pixel: give that device a colour source at the point cloud '
			'resolution, or turn Use coloured ICP off.')
	return colors[mask]


def makeCloud(o3d, points, colors=None):
	cloud = o3d.geometry.PointCloud()
	cloud.points = o3d.utility.Vector3dVector(points)
	if colors is not None and len(colors):
		cloud.colors = o3d.utility.Vector3dVector(colors)
	return cloud


def loadClouds(o3d, job, colored):
	"""
	Read both .npy clouds, drop the unusable points, and build the open3d
	clouds, with colours attached when coloured ICP asked for them. Returns
	(target, source, targetCount, sourceCount): the counts say how many points
	survived the filter and go back in the reply.
	"""
	maxRange = float(job.get('maxRange', 0.0))
	targetPts, targetMask = loadPoints(job['target'], maxRange)
	sourcePts, sourceMask = loadPoints(job['source'], maxRange)
	targetPts = targetPts[targetMask]
	sourcePts = sourcePts[sourceMask]
	if not len(targetPts) or not len(sourcePts):
		raise ValueError('a cloud has no valid points after filtering '
			f'(target {len(targetPts)}, source {len(sourcePts)})')

	targetCol = sourceCol = None
	if colored:
		if not job.get('targetColors') or not job.get('sourceColors'):
			raise ValueError('coloured ICP needs colours for both clouds')
		targetCol = loadColors(job['targetColors'], targetMask, 'target')
		sourceCol = loadColors(job['sourceColors'], sourceMask, 'source')

	target = makeCloud(o3d, targetPts, targetCol)
	source = makeCloud(o3d, sourcePts, sourceCol)
	return target, source, len(targetPts), len(sourcePts)


def downsample(cloud, voxel):
	"""A voxel of 0 means leave the cloud alone, at full detail."""
	return cloud.voxel_down_sample(voxel) if voxel and voxel > 0 else cloud


def estimateNormals(o3d, cloud, radius, maxNn=30):
	"""
	Work out which way each bit of surface faces. Open3D leaves the direction
	arbitrary and the matching cares, so point them all at the camera: every
	cloud arrives in its own camera's space, with the camera at the origin.
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
	The rough match, and the first guess: repeatedly pick three source points
	and pair them with the target points that look most like them. The checkers
	throw out bad pairings early.

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
		# Open3D's own default. It stops on confidence well before the try
		# limit, so a bigger limit buys nothing.
		criteria=reg.RANSACConvergenceCriteria(100000, 0.999))


def poseGap(a, b):
	"""
	How far apart two answers are: metres, plus degrees counted at 0.05 m each,
	so one degree weighs about as much as five centimetres. Only used to compare
	answers with each other, never reported.
	"""
	d = np.linalg.inv(a) @ b
	turn = np.degrees(np.arccos(np.clip((np.trace(d[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)))
	return float(np.linalg.norm(d[:3, 3]) + 0.05 * turn)


def consensusGlobal(o3d, sourceDown, targetDown, sourceFpfh, targetFpfh, voxel, runs):
	"""
	Run the rough match several times and keep the answer the others agree with.

	This catches the failure that matters: an answer metres out that still
	scores well. The score cannot see it, because a wrong alignment of two
	clouds that both contain a floor still puts most points near some surface.
	How far the runs land apart does see it, since the matcher disagrees with
	itself exactly when the scene is ambiguous.

	Returns the answer closest to all the others, not the first or the best
	scoring one. Picking by score would defeat the point.
	"""
	results = [globalRegistration(o3d, sourceDown, targetDown,
		sourceFpfh, targetFpfh, voxel) for _ in range(int(runs))]
	poses = [np.array(r.transformation, dtype=np.float64) for r in results]
	gaps = [[poseGap(p, q) for q in poses] for p in poses]
	best = int(np.argmin([sum(row) for row in gaps]))
	pairs = [gaps[i][j] for i in range(len(poses)) for j in range(i + 1, len(poses))]
	return results[best], float(np.median(pairs))


def icpRefine(o3d, source, target, distance, matrix, colored=False, maxIterations=50):
	"""
	The refine, starting from matrix. Generalized ICP by default: it looks at
	the shape of the surface around a point at both ends rather than the way it
	faces at one, which is what partly overlapping views need, and it works that
	out itself, so the source needs no normals.

	Coloured ICP is point based instead. It needs normals on the target and
	colours on both clouds.
	"""
	reg = o3d.pipelines.registration
	criteria = reg.ICPConvergenceCriteria(max_iteration=int(maxIterations))
	if colored:
		return reg.registration_colored_icp(
			source, target, distance, matrix,
			reg.TransformationEstimationForColoredICP(), criteria)
	return reg.registration_generalized_icp(
		source, target, distance, matrix,
		reg.TransformationEstimationForGeneralizedICP(), criteria)


# ___________________________________________________________________ grading


def grade(result, distance, stage):
	"""Turn a registration result into numbers plus a verdict."""
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


def overlapFraction(o3d, source, target, matrix,
		radius=OVERLAP_RADIUS, samples=OVERLAP_SAMPLES):
	"""
	How much of what the target camera sees the source camera sees too, as a
	fraction, once matrix has been applied.

	The sample is taken with a stride rather than at random, so the answer
	repeats without needing a seed. It is pushed through the inverse matrix into
	the source's own space instead of moving the source cloud: the transform is
	rigid, so the distances are the same either way, and this copies 4000 points
	rather than all of them.

	Returns None rather than raising. A calibration that worked must not be
	thrown away because the number describing it could not be worked out.
	"""
	try:
		points = np.asarray(target.points)
		if len(points) > samples:
			points = points[::max(1, len(points) // samples)][:samples]
		if not len(points):
			return None
		inverse = np.linalg.inv(np.asarray(matrix, dtype=np.float64))
		points = points @ inverse[:3, :3].T + inverse[:3, 3]
		probe = o3d.geometry.PointCloud()
		probe.points = o3d.utility.Vector3dVector(points)
		distances = np.asarray(probe.compute_point_cloud_distance(source))
		return float((distances < radius).mean())
	except Exception:
		return None


def applyConsensus(metrics, consensus, limit):
	"""
	Stamp how far apart the repeated rough matches landed onto the metrics.
	Runs that disagree with each other are not to be trusted however well they
	score, so disagreement downgrades an OK to WARN. It only ever warns: it
	never turns a failure into something better.
	"""
	agreed = consensus <= limit
	metrics['consensus'] = float(consensus)
	metrics['consensusLimit'] = float(limit)
	metrics['agreed'] = agreed
	if not agreed and metrics['status'] == 'OK':
		metrics['status'] = 'WARN'


# ____________________________________________________________________ stages


def globalStage(o3d, source, target, voxel, seed, runs):
	"""
	The rough match from nothing: downsample, describe, align by features.
	Returns the matrix and its metrics; when the match ran more than once the
	metrics also say whether the runs agreed (see consensusGlobal).
	"""
	targetDown, targetFpfh = featureCloud(o3d, target, voxel)
	sourceDown, sourceFpfh = featureCloud(o3d, source, voxel)
	if runs > 1:
		result, spread = consensusGlobal(o3d, sourceDown, targetDown,
			sourceFpfh, targetFpfh, voxel, runs)
	else:
		result = globalRegistration(o3d, sourceDown, targetDown,
			sourceFpfh, targetFpfh, voxel, seed)
		spread = None
	metrics = grade(result, voxel * 1.5, 'global')
	if spread is not None:
		applyConsensus(metrics, spread, voxel * CONSENSUS_WARN_FACTOR)
	# result.transformation is read only and laid out the other way round;
	# np.array copies it into a plain row-major matrix.
	return np.array(result.transformation, dtype=np.float64), metrics


def refineStage(o3d, source, target, voxel, refineVoxel, init, colored):
	"""
	ICP from init. Only the target gets normals: coloured ICP needs them there,
	and generalized ICP works out its own, so the source needs none (see
	icpRefine).
	"""
	targetFine = downsample(target, refineVoxel)
	sourceFine = downsample(source, refineVoxel)
	estimateNormals(o3d, targetFine, radius=(refineVoxel or voxel) * 2)
	distance = (refineVoxel or voxel) * 2.5
	result = icpRefine(o3d, sourceFine, targetFine, distance, init, colored)
	metrics = grade(result, distance, 'coloredIcp' if colored else 'icp')
	# Same read-only, transposed-layout matrix as in globalStage: copy it.
	return np.array(result.transformation, dtype=np.float64), metrics


def mergeGlobalMetrics(metrics, globalMetrics):
	"""
	Fold the rough match's numbers into the refined reply. The refine polishes
	whatever it was handed, so its own score says nothing about whether the
	rough match started in the right place. Carry the disagreement up too, or
	the warning dies at the refine and the user sees OK.
	"""
	metrics['stage'] = 'globalThenIcp'
	metrics['global'] = globalMetrics
	if 'consensus' in globalMetrics:
		applyConsensus(metrics, globalMetrics['consensus'],
			globalMetrics['consensusLimit'])


# _____________________________________________________________________ entry


def reply(matrix, metrics, targetCount, sourceCount, overlap=None):
	out = {'ok': True, 'matrix': [float(v) for v in np.asarray(matrix).reshape(-1)]}
	out.update(metrics)
	out['targetPoints'] = int(targetCount)
	out['sourcePoints'] = int(sourceCount)
	# Left out entirely when it could not be measured, rather than sent as a
	# zero that reads like two cameras sharing nothing.
	if overlap is not None:
		out['overlap'] = float(overlap)
	return out


def run(job):
	# The rough match runs on several threads at once and they race, so seeding
	# open3d alone does not make a run repeat. Running on one thread does. Only
	# done when a seed was asked for, because it costs the speed, and it has to
	# happen before the import, which is why open3d is imported here rather
	# than at the top of the file.
	seed = int(job.get('seed', -1))
	if seed >= 0:
		os.environ['OMP_NUM_THREADS'] = '1'

	import open3d as o3d

	mode = job.get('mode', 'global')
	voxel = float(job.get('voxel', 0.05))
	refineVoxel = float(job.get('refineVoxel', 0.01))
	colored = bool(job.get('colored', False))

	target, source, targetCount, sourceCount = loadClouds(o3d, job, colored)

	globalMetrics = None
	matrix = metrics = None
	if mode in ('global', 'globalThenIcp'):
		# A seeded run comes out the same every time, so asking four of them
		# whether they agree answers nothing: a seed turns the consensus check
		# off, and the reply then carries no consensus keys at all.
		runs = 1 if seed >= 0 else int(job.get('consensusRuns', 4))
		init, globalMetrics = globalStage(o3d, source, target, voxel, seed, runs)
		# Refining a failed rough match just polishes a wrong answer into a
		# confident looking one. Hand the failure back instead.
		if mode == 'global' or globalMetrics['status'] == 'FAIL':
			matrix, metrics = init, globalMetrics
	else:
		init = job.get('init')
		init = np.eye(4) if not init else np.array(init, dtype=np.float64).reshape(4, 4)

	if metrics is None:
		matrix, metrics = refineStage(o3d, source, target, voxel, refineVoxel, init, colored)
		if globalMetrics is not None:
			mergeGlobalMetrics(metrics, globalMetrics)

	# One exit, so the overlap is measured once, against whatever matrix is
	# actually being handed back.
	return reply(matrix, metrics, targetCount, sourceCount,
		overlapFraction(o3d, source, target, matrix))


def probe():
	info = {'ok': True, 'python': sys.version.split()[0], 'executable': sys.executable}
	try:
		info['numpy'] = np.__version__
	except Exception as err:
		info['numpy'] = f'error: {err}'
	try:
		import open3d as o3d
		info['open3d'] = o3d.__version__
	except Exception as err:
		info['ok'] = False
		info['open3d'] = None
		info['error'] = f'{type(err).__name__}: {err}'
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
		out = {'ok': False, 'error': f'{type(err).__name__}: {err}',
			'traceback': traceback.format_exc()}

	payload = json.dumps(out)
	if resultPath:
		with open(resultPath, 'w', encoding='utf-8') as handle:
			handle.write(payload)
	print(payload)
	return 0 if out.get('ok') else 1


# A TouchDesigner DAT also runs with __name__ == '__main__', so the plain guard
# is not enough: reading this file as a DAT module would run main(), pick up
# TouchDesigner's own arguments and wedge it. Check we were really launched as
# this script.
if __name__ == '__main__' and os.path.basename(sys.argv[0] or '').startswith('worker'):
	sys.exit(main())
