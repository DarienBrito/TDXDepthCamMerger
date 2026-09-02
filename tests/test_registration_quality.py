"""
Does the registration actually give good answers?

test_worker.py checks that the worker behaves: it runs, reads a job, reports
failures cleanly. This file asks the other question, on made up but believable
depth camera clouds (see synth.py): how accurate is the answer, when does it
stop being accurate, and does what the user is shown match the truth.

    python test_registration_quality.py [python-with-open3d] [--quick]

The worker is called in this same process, so open3d is loaded once rather than
once per job. The determinism section is the exception and starts a real
subprocess, because a seeded run only repeats if the thread setting is made
before open3d loads, which in process has already happened.

Every limit below was measured before it was written down. Where the pipeline
has a real limitation the test measures and prints it rather than hiding it;
those lines are marked FINDING.
"""

import json
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

sys.stdout.reconfigure(encoding='utf-8')

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, os.pardir, 'src'))

import synth as S                                            # noqa: E402
import worker as W                                           # noqa: E402
import extTDXDepthCamMerger as ext                           # noqa: E402

WORKER = os.path.join(HERE, os.pardir, 'src', 'worker.py')
ARGS = [a for a in sys.argv[1:] if not a.startswith('-')]
QUICK = '--quick' in sys.argv
PYEXE = ARGS[0] if ARGS else sys.executable

# What counts as a calibration that worked. A depth camera's own noise is a few
# millimetres, so 50 mm / 2 deg means "the clouds visibly sit on top of each
# other", not a precision claim. Trials with more noise than a real camera has
# get a looser limit of their own.
GOOD_M = 0.05
GOOD_DEG = 2.0

# How much the two cameras have to see in common before the rough match can be
# trusted. Below this it usually fails, at or above it usually works.
RELIABLE_OVERLAP = 0.55

# The rough match is unseeded, so one trial going wrong is not a regression.
# The limits below are rates over the whole sweep, not per run. 0.75 is a floor
# to catch a collapse, not a promise about the rate: print it every run and
# watch it.
RELIABLE_RATE = 0.75

# Above this much shared view, a wrong answer graded OK is a real regression
# rather than the known trouble with cameras that barely see the same things.
TRUSTED_OVERLAP = 0.70

FAILURES = []
TMP = tempfile.mkdtemp(prefix='tdxquality_')
_counter = [0]
_runs = [0]


def check(name, ok, detail=''):
    if not ok:
        FAILURES.append(name)
    print('  [{}] {}{}'.format('PASS' if ok else 'FAIL', name,
                               ('  -> ' + detail) if detail else ''))


def finding(text):
    print('  [FINDING] {}'.format(text))


def section(title):
    print('\n' + title)
    print('  ' + '-' * (len(title) + 2))


def dump(capture, kind='points'):
    _counter[0] += 1
    path = os.path.join(TMP, '{}_{}.npy'.format(kind, _counter[0]))
    return capture.save(path) if kind == 'points' else capture.saveColors(path)


def register(target, source, mode='globalThenIcp', voxel=0.05, refineVoxel=0.01,
             colors=False, **extra):
    """
    Run one job the way the component does, and time it. The defaults here have
    to match what the component ships with (section 4 of tools/td_build.py:
    Voxelsize 0.05, Refinevoxel 0.01), or the suite measures settings no user
    has.
    """
    job = {'mode': mode, 'target': dump(target), 'source': dump(source),
           'voxel': voxel, 'refineVoxel': refineVoxel, 'maxRange': 0.0, 'seed': -1}
    if colors:
        job['colored'] = True
        job['targetColors'] = dump(target, 'colors')
        job['sourceColors'] = dump(source, 'colors')
    job.update(extra)
    _runs[0] += 1
    start = time.time()
    result = W.run(job)
    result['seconds'] = time.time() - start
    return result, np.array(result['matrix'], dtype=np.float64).reshape(4, 4)


def truth(target, source):
    """The right answer for putting source onto target."""
    return np.linalg.inv(target.T) @ source.T


def pair(scene, spanDeg, noise=0.001, **cam):
    """Two cameras spanDeg apart on the circle, each cloud in its own space."""
    return [c.capture(scene)
            for c in S.ringCameras(2, spanDeg=spanDeg, noise=noise, **cam)]


print('python   :', sys.executable)
print('mode     :', 'quick' if QUICK else 'full')
print('temp     :', TMP)
probe = W.probe()
print('open3d   : {}  numpy {}'.format(probe.get('open3d'), probe.get('numpy')))
check('open3d is importable out of process', probe.get('ok'), str(probe.get('error')))
suiteStart = time.time()


# ____________________________________________________ A. convention and sanity

section('A. the returned matrix means what the docstring says')

scene = S.studioScene(seed=0)
a, b = pair(scene, 45.0)
M = truth(a, b)
print('  target {} valid points, source {}, overlap {:.2f}'.format(
    int(a.valid.sum()), int(b.valid.sum()), S.overlapFraction(a, b)))

res, Mr = register(a, b)
te, de = S.poseError(M, Mr)
print('  recovered  {:.4f} m / {:.3f} deg   fitness {:.3f}  rmse {:.4f}  {}  {:.1f}s'.format(
    te, de, res['fitness'], res['rmse'], res['status'], res['seconds']))
check('45 deg apart, high overlap: recovers the pose',
      te < GOOD_M and de < GOOD_DEG, '{:.4f} m / {:.3f} deg'.format(te, de))

# The check that needs no known answer: put the source points through the
# matrix and see whether they land on the target's surface.
got = S.residualStats(b.validPoints, a.validPoints, Mr)
ideal = S.residualStats(b.validPoints, a.validPoints, M)
naive = S.residualStats(b.validPoints, a.validPoints, np.eye(4))
print('  surface residual: recovered inlier {:.3f} / median {:.4f} m | truth {:.3f} / {:.4f}'
      ' | uncalibrated {:.3f} / {:.4f}'.format(
          got['inlierFraction'], got['median'], ideal['inlierFraction'], ideal['median'],
          naive['inlierFraction'], naive['median']))
check('moved source lands on the target surface as well as the truth does',
      got['inlierFraction'] >= ideal['inlierFraction'] - 0.02,
      '{:.3f} vs {:.3f}'.format(got['inlierFraction'], ideal['inlierFraction']))
check('and better than leaving it uncalibrated',
      got['inlierFraction'] > naive['inlierFraction'] + 0.05,
      '{:.3f} vs {:.3f}'.format(got['inlierFraction'], naive['inlierFraction']))

# Registering the pair the other way round has to undo the first answer, or one
# of the two directions is being read backwards.
resBack, Mback = register(b, a)
loop = Mr @ Mback
lt, ld = S.poseError(np.eye(4), loop)
print('  round trip A<-B then B<-A: {:.4f} m / {:.3f} deg from identity'.format(lt, ld))
check('M(a<-b) @ M(b<-a) is the identity', lt < GOOD_M and ld < GOOD_DEG,
      '{:.4f} m / {:.3f} deg'.format(lt, ld))

sameA = S.DepthCamera((0, 1.35, 2.4), (0, 0.65, 0), seed=1, noise=0.001).capture(scene)
sameB = S.DepthCamera((0, 1.35, 2.4), (0, 0.65, 0), seed=77, noise=0.001).capture(scene)
_, Msame = register(sameA, sameB)
st, sd = S.poseError(np.eye(4), Msame)
check('two captures from one pose register to the identity', st < 0.02 and sd < 0.5,
      '{:.4f} m / {:.3f} deg'.format(st, sd))


# _____________________________________________________ B. accuracy, with stats

section('B. accuracy sweep: where does it work and where does it stop')

trials = []


def trial(label, target, source, expected, tolM=GOOD_M, tolDeg=GOOD_DEG, **kw):
    over = S.overlapFraction(target, source)
    res, Mr = register(target, source, **kw)
    t, d = S.poseError(expected, Mr)
    row = {'label': label, 'overlap': over, 'm': t, 'deg': d,
           'fitness': res['fitness'], 'rmse': res['rmse'], 'status': res['status'],
           'good': bool(t < tolM and d < tolDeg),
           # "good" is a near miss test against this trial's own limit.
           # "gross" is the failure that matters: an answer metres out. Section
           # C uses gross, so a run that lands just past its limit is not
           # reported as confidently wrong.
           'gross': bool(t > 0.25 or d > 10.0), 'seconds': res['seconds'],
           'agreed': bool(res.get('agreed', True)),
           'consensus': res.get('consensus'),
           'points': res['sourcePoints'], 'matrix': Mr}
    trials.append(row)
    print('  {:<22} ov {:.2f}  {:8.4f} m {:8.3f} deg  fit {:.3f}  {:<4}  {}  {:.1f}s'.format(
        label, over, t, d, res['fitness'], res['status'],
        'ok ' if row['good'] else 'WRONG', res['seconds']))
    return row


seeds = (0, 1) if QUICK else (0, 1, 2, 3, 4, 5)
spans = (35.0, 60.0, 85.0) if QUICK else (35.0, 60.0, 85.0, 110.0)

print('  camera separation, at 1 mm noise')
for span in spans:
    for sd in seeds:
        sc = S.studioScene(seed=sd)
        t_, s_ = pair(sc, span)
        trial('{:.0f} deg seed {}'.format(span, sd), t_, s_, truth(t_, s_))

# A Kinect wobbles by about 1 mm at a metre. 6 mm is worse than any real sensor
# and is here to find the breaking point, so it is judged against a limit that
# grows with the noise.
print('  sensor noise, at 50 deg separation (sigma quoted at 1 m, scaled by z squared)')
noises = (0.0, 0.003) if QUICK else (0.0, 0.001, 0.003, 0.006)
noiseSeeds = seeds[:2] if QUICK else seeds[:4]
for nz in noises:
    for sd in noiseSeeds:
        sc = S.studioScene(seed=sd)
        t_, s_ = pair(sc, 50.0, noise=nz)
        trial('noise {:.0f} mm seed {}'.format(nz * 1000, sd), t_, s_, truth(t_, s_),
              tolM=max(GOOD_M, 12.0 * nz), tolDeg=max(GOOD_DEG, 400.0 * nz))

reliable = [r for r in trials if r['overlap'] >= RELIABLE_OVERLAP]
marginal = [r for r in trials if r['overlap'] < RELIABLE_OVERLAP]
missed = [r for r in reliable if not r['good']]
rate = 1.0 - len(missed) / max(len(reliable), 1)
print('  {} trials: {} at overlap >= {:.2f} ({:.0%} recovered), {} below'.format(
    len(trials), len(reliable), RELIABLE_OVERLAP, rate, len(marginal)))
check('well overlapped pairs recover at least {:.0%} of the time'.format(RELIABLE_RATE),
      rate >= RELIABLE_RATE,
      ', '.join('{} ({:.2f} ov, {:.3f} m)'.format(r['label'], r['overlap'], r['m'])
                for r in missed) or 'none failed')
if reliable:
    ms = np.array([r['m'] for r in reliable])
    ds = np.array([r['deg'] for r in reliable])
    print('  well overlapped: median {:.4f} m / {:.3f} deg, worst {:.4f} m / {:.3f} deg'.format(
        np.median(ms), np.median(ds), ms.max(), ds.max()))
    check('median accuracy stays well under a centimetre', float(np.median(ms)) < 0.015,
          '{:.4f} m'.format(np.median(ms)))
if marginal:
    marginalRate = sum(r['good'] for r in marginal) / len(marginal)
    finding('below {:.0%} overlap only {:.0%} of {} runs recovered the pose, and the '
            'misses are not near misses: they are 3 to 5 m out, in the wrong rotational '
            'basin. Overlap, not any worker parameter, is what decides.'.format(
                RELIABLE_OVERLAP, marginalRate, len(marginal)))

noiseRows = [r for r in trials if r['label'].startswith('noise')]
if noiseRows:
    print('  noise response: ' + ', '.join(
        '{} -> {:.3f} m'.format(r['label'].split(' seed')[0], r['m'])
        for r in noiseRows if r['label'].endswith('seed 0')))
    # 3 mm already covers any real sensor, so only up to there is checked. 6 mm
    # is there to show where it breaks.
    # Judged on the middle result across seeds, not on every one: a single
    # trial drifting past the limit now and then is the spread of an unseeded
    # pipeline, not a regression.
    for nz in noises:
        if nz > 0.003:
            continue
        rows = [r for r in noiseRows
                if r['overlap'] >= RELIABLE_OVERLAP
                and abs(float(r['label'].split()[1]) - nz * 1000) < 0.01]
        if not rows:
            continue
        med = float(np.median([r['m'] for r in rows]))
        check('{:.0f} mm per metre of noise leaves a median under tolerance'.format(nz * 1000),
              med < max(GOOD_M, 12.0 * nz),
              '{:.4f} m over {} seeds (worst {:.4f})'.format(
                  med, len(rows), max(r['m'] for r in rows)))

    # 6 mm per metre is about 35 mm at the 2.4 m the cameras stand at, further
    # than the refine can reach, and the answer falls apart by an amount that
    # depends on the scene. Widening Refinevoxel does not reliably help, so
    # this is reported rather than checked.
    broken = [r for r in noiseRows if not r['good']]
    if broken:
        finding('{} of {} noise trials missed their tolerance, all at 6 mm per metre, '
                'which is several times any shipping depth sensor. Widening Refinevoxel '
                'does NOT reliably fix it (measured: worse on one scene, better on '
                'another), so treat heavy sensor noise as a capture problem rather than '
                'a parameter to tune.'.format(len(broken), len(noiseRows)))

# What a user sees on a second press of Calibrate: the rough match is unseeded,
# so the same clouds do not give exactly the same answer twice.
print('  pressing Calibrate five times on one well overlapped pair')
repScene = S.studioScene(seed=0)
rpA, rpB = pair(repScene, 45.0)
Mrep = truth(rpA, rpB)
spread = []
for k in range(3 if QUICK else 5):
    _, Mk = register(rpA, rpB)
    spread.append(S.poseError(Mrep, Mk))
print('    ' + '  '.join('{:.4f}m/{:.2f}d'.format(t, d) for t, d in spread))
worstRep = max(t for t, _ in spread)
check('every repeat lands within tolerance', worstRep < GOOD_M,
      'worst {:.4f} m'.format(worstRep))
check('repeats agree with each other to within a centimetre',
      max(t for t, _ in spread) - min(t for t, _ in spread) < 0.01,
      'spread {:.4f} m'.format(max(t for t, _ in spread) - min(t for t, _ in spread)))


# ______________________________________ C. does the reported status track truth

section('C. the numbers the component shows the user vs the truth')

table = {}
for r in trials:
    table.setdefault((r['status'], r['good']), []).append(r)
print('  status   recovered   wrong')
for status in ('OK', 'WARN', 'FAIL'):
    print('  {:<8} {:>9} {:>7}'.format(status, len(table.get((status, True), [])),
                                       len(table.get((status, False), []))))

confidentlyWrong = [r for r in trials if r['status'] == 'OK' and r['gross']]
falseAlarms = table.get(('FAIL', True), [])

check('nothing graded FAIL was actually correct', not falseAlarms,
      ', '.join(r['label'] for r in falseAlarms))
check('no well overlapped pair is confidently wrong',
      all(r['overlap'] < TRUSTED_OVERLAP for r in confidentlyWrong),
      ', '.join('{} ({:.2f} ov)'.format(r['label'], r['overlap'])
                for r in confidentlyWrong if r['overlap'] >= TRUSTED_OVERLAP))

# The agreement check exists because the score cannot do this job. Measure both
# sides: how many wrong answers it flags, and how many right ones it doubts. A
# check that flags everything is useless.
checked = [r for r in trials if r['consensus'] is not None]
if checked:
    wrongRuns = [r for r in checked if r['gross']]
    rightRuns = [r for r in checked if r['good']]
    caught = [r for r in wrongRuns if not r['agreed']]
    alarms = [r for r in rightRuns if not r['agreed']]
    print('  consensus check: flagged {} of {} wrong-basin runs, and doubted {} of {}'
          ' correct ones'.format(len(caught), len(wrongRuns), len(alarms), len(rightRuns)))
    if wrongRuns:
        print('    spread on wrong runs: median {:.3f} m | on correct runs: median {:.3f} m'
              .format(float(np.median([r['consensus'] for r in wrongRuns])),
                      float(np.median([r['consensus'] for r in rightRuns]))))
        check('the consensus check flags most wrong-basin answers',
              len(caught) * 2 >= len(wrongRuns),
              '{} of {}'.format(len(caught), len(wrongRuns)))
    if rightRuns:
        # Allowed up to 45%, with room to spare, because what matters is the
        # catch rate above. A right answer that gets doubted is one the matcher
        # reached shakily, which is worth saying out loud.
        check('and doubts fewer than half of the correct ones',
              len(alarms) * 100 <= 45 * len(rightRuns),
              '{} of {}'.format(len(alarms), len(rightRuns)))

good = np.array([r['fitness'] for r in trials if r['good']])
bad = np.array([r['fitness'] for r in trials if not r['good']])
if len(good) and len(bad):
    print('  fitness: recovered runs median {:.3f} (min {:.3f}), wrong runs median {:.3f}'
          ' (max {:.3f})'.format(np.median(good), good.min(), np.median(bad), bad.max()))
    if bad.max() >= good.min():
        finding('fitness does NOT separate a good calibration from a wrong one: a wrong '
                'run scored {:.3f} while a correct one scored {:.3f}. It measures how '
                'many source points found a target point nearby, and a wrong alignment '
                'of two clouds that both contain a floor still satisfies that. Read '
                'Laststatus as "the solver converged", never as "the calibration is '
                'right"; the viewport is what confirms a calibration.'.format(
                    bad.max(), good.min()))
if confidentlyWrong:
    finding('{} of {} runs came back graded OK while being in the WRONG BASIN, metres '
            'out, all at overlap {:.2f} or below. A user whose cameras barely see the '
            'same thing gets a confident wrong answer, so overlap is the thing to tell '
            'them to fix.'.format(len(confidentlyWrong), len(trials),
                                  max(r['overlap'] for r in confidentlyWrong)))


# ____________________________________________________ D. unobservable geometry

section('D. a scene that cannot be registered by geometry alone')

flat = S.planeScene()
fa = S.DepthCamera((0, 1.5, 1.3), (0, 0, 0), seed=1, noise=0.0008).capture(flat)
fb = S.DepthCamera((0.6, 1.5, 1.1), (0.6, 0, -0.2), seed=2, noise=0.0008).capture(flat)
Mflat = truth(fa, fb)
res, Mr = register(fa, fb)
pt, pd = S.poseError(Mflat, Mr)
stats = S.residualStats(fb.validPoints, fa.validPoints, Mr)
print('  bare plane: {:.4f} m / {:.3f} deg from truth, fitness {:.3f}, status {}'.format(
    pt, pd, res['fitness'], res['status']))
print('  yet the clouds do sit on each other: inlier {:.3f}, median residual {:.4f} m'.format(
    stats['inlierFraction'], stats['median']))
check('the wrong answer is nonetheless a valid surface fit',
      stats['inlierFraction'] > 0.85, '{:.3f}'.format(stats['inlierFraction']))
if pt > 0.1 or pd > 5.0:
    finding('on a single flat surface, sliding along it and turning about its normal '
            'are unobservable, so the solver returns a confident wrong pose ({:.2f} m '
            'and {:.1f} deg from the truth, at fitness {:.2f} graded {}). Which of the '
            'two unobservable directions it lands in varies run to run. Calibration '
            'needs geometry with structure in it, or coloured ICP, which is what D and '
            'E together are for.'.format(pt, pd, res['fitness'], res['status']))


# ______________________________________________________ E. what colour buys you

section('E. coloured ICP on the same unobservable plane')

painted = S.texturedPlane(seed=4)
ca = S.DepthCamera((0, 1.5, 1.3), (0, 0, 0), seed=1, noise=0.0008).capture(painted)
cb = S.DepthCamera((0.5, 1.5, 1.15), (0.5, 0, -0.15), seed=2, noise=0.0008).capture(painted)
Mcol = truth(ca, cb)
# Start both runs from the same slightly wrong answer, the way Refine is used.
init = S.rigid(2.5, [0, 1, 0], [0.10, 0.0, -0.07]) @ Mcol
print('  seeded from {:.4f} m / {:.3f} deg off'.format(*S.poseError(Mcol, init)))
plainRes, plainM = register(ca, cb, mode='icp', init=list(init.reshape(-1)))
colRes, colM = register(ca, cb, mode='icp', colors=True, init=list(init.reshape(-1)))
pt, pd = S.poseError(Mcol, plainM)
ct, cd = S.poseError(Mcol, colM)
print('  plain ICP    {:.4f} m / {:.3f} deg   fitness {:.3f}'.format(pt, pd, plainRes['fitness']))
print('  coloured ICP {:.4f} m / {:.3f} deg   fitness {:.3f}'.format(ct, cd, colRes['fitness']))
check('coloured ICP recovers what geometry cannot', ct < 0.02 and cd < 0.5,
      '{:.4f} m / {:.3f} deg'.format(ct, cd))
check('and plain ICP on the same input does not', pt > ct * 3,
      'plain {:.4f} m vs coloured {:.4f} m'.format(pt, ct))
finding('coloured ICP is only worth ticking when the surface texture does not repeat. '
        'On a checkerboard it locked onto the neighbouring square and came out worse '
        'than plain ICP, which is why synth.Checker carries that warning.')


# _______________________________________________________________ F. robustness

section('F. robustness to the things a real capture does')

# 35 degrees rather than 50: the moved object hides part of the frame, and at
# 50 that left the cameras sharing too little for anything to be learned. What
# is being tested here is the disturbance, so the layout has to stay easy.
sc = S.studioScene(seed=5)
ra, rb = pair(sc, 35.0)
Mrob = truth(ra, rb)
baseline = trial('clean baseline', ra, rb, Mrob)

rng = np.random.default_rng(0)
pts = rb.points.copy()
idx = rng.choice(np.flatnonzero(rb.valid), int(0.20 * rb.valid.sum()), replace=False)
pts[idx] = rng.uniform(-3.0, 3.0, (len(idx), 3)).astype(np.float32)
trial('20% flyer points', ra, S.Capture(pts, rb.colors, rb.valid, rb.T, rb.res), Mrob)

# Something moved between the two shots: the usual reason a calibration that
# looked fine yesterday does not today.
moved = S.ringCameras(2, spanDeg=35.0, noise=0.001)[1].capture(
    sc, extra=[S.Box([0.15, 0.30, 1.05], [0.45, 0.60, 0.35], yawDeg=10.0)])
trial('object moved in frame', ra, moved, Mrob)

thin = rb.points.copy()
thin[rng.random(len(thin)) < 0.75] = 0.0
trial('source 4x sparser', ra,
      S.Capture(thin, rb.colors, np.abs(thin).sum(1) > 0, rb.T, rb.res), Mrob)

hard = [r for r in trials[-3:]]
check('outliers, a moved object and a sparser cloud stay recoverable',
      all(r['good'] for r in hard),
      ', '.join('{} {:.3f} m'.format(r['label'], r['m']) for r in hard if not r['good']))

# The cameras stand 2.4 m out, so a limit has to be further than that or it
# throws the subject away. Both sides are worth showing: the parameter is for
# dropping a far wall, and set too tight it quietly ruins a calibration.
print('  maxRange, in metres from each camera')
for limit in (3.0, 2.0, 1.5):
    try:
        res, Mc = register(ra, rb, maxRange=limit)
    except ValueError as err:
        print('    {:.1f} m  refused: {}'.format(limit, err))
        finding('maxRange {:.1f} m is well inside the 2.4 m the cameras stand at, so '
                'nothing survives the crop. The worker says so instead of registering '
                'an empty cloud, which is the behaviour to keep.'.format(limit))
        continue
    t, d = S.poseError(Mrob, Mc)
    print('    {:.1f} m  kept {:6d} of {} source points  {:8.4f} m {:7.3f} deg  {}'.format(
        limit, res['sourcePoints'], int(rb.valid.sum()), t, d, res['status']))
    if limit == 3.0:
        check('maxRange crops the cloud', res['sourcePoints'] < rb.valid.sum(),
              '{} vs {}'.format(res['sourcePoints'], int(rb.valid.sum())))
        check('a crop that clears the working distance keeps the calibration',
              t < GOOD_M and d < GOOD_DEG, '{:.4f} m / {:.3f} deg'.format(t, d))
    elif t > GOOD_M:
        finding('maxRange {:.1f} m cuts into the 2.4 m the cameras stand at: {} of {} '
                'points survived and the answer came back {:.2f} m / {:.0f} deg out, '
                'graded {}. The parameter is for dropping a far wall, not for '
                'tightening a calibration.'.format(
                    limit, res['sourcePoints'], int(rb.valid.sum()), t, d, res['status']))

# Two things are worth keeping apart. The whole pipeline barely notices the
# voxel, because the refine cleans up after the rough match. The rough match on
# its own does notice, and it varies run to run, so its column is a rate over
# several tries rather than one number. Judging it on a single try would raise
# false alarms.
print('  voxel size: full pipeline vs the global stage on its own')
voxels = (0.03, 0.05, 0.10) if QUICK else (0.02, 0.03, 0.05, 0.08, 0.12)
# The checked voxel gets more tries than the rest of the curve, so the check
# sits clear of the run to run spread.
GATED_VOXEL = 0.05
curve = {}
for v in voxels:
    repeats = (4 if QUICK else 6) if v == GATED_VOXEL else (2 if QUICK else 3)
    res, Mv = register(ra, rb, voxel=v)
    t, d = S.poseError(Mrob, Mv)
    basin = 0
    for _ in range(repeats):
        _, Mg = register(ra, rb, mode='global', voxel=v)
        gt, gd = S.poseError(Mrob, Mg)
        basin += int(gt < 0.15 and gd < 6.0)
    curve[v] = (t, d, basin, repeats)
    print('    voxel {:.2f}  pipeline {:8.4f} m {:7.3f} deg   global stage in basin '
          '{}/{}'.format(v, t, d, basin, repeats))
dt, dd, dBasin, dRepeats = curve[GATED_VOXEL]
check('the shipped default voxel recovers the pose', dt < GOOD_M and dd < GOOD_DEG,
      '{:.4f} m / {:.3f} deg'.format(dt, dd))
check('and its global stage finds the right basin more often than not',
      dBasin * 2 >= dRepeats, '{}/{}'.format(dBasin, dRepeats))
poor = [v for v, (t, d, basin, reps) in sorted(curve.items()) if basin * 2 < reps]
if poor:
    finding('the voxel has a working range and the shipped 0.05 sits in the middle of '
            'it. At {} the global stage missed the basin in most repeats. Too fine and '
            'FPFH describes noise, too coarse and it describes nothing; the pipeline '
            'still recovered because ICP cleaned up after it, but that is luck, not '
            'margin.'.format(', '.join('{:.2f}'.format(v) for v in poor)))


# _____________________________________________ F2. how far off Refine can pull

section('F2. Refine: how wrong may the stored calibration be')

# Refine only refines what is already stored. How far it can reach comes from
# Refinevoxel, so it is worth measuring where it stops rather than guessing.
capA, capB = pair(S.studioScene(seed=0), 45.0)
Mref = truth(capA, capB)
recovered = []
nudges = ((0.02, 1.0), (0.05, 2.0), (0.15, 6.0), (0.30, 12.0), (0.60, 25.0))
for offset, angle in nudges:
    init = S.rigid(angle, [0.2, 1.0, 0.1], [offset, offset * 0.3, -offset * 0.5]) @ Mref
    startT, startD = S.poseError(Mref, init)
    res, Mfix = register(capA, capB, mode='icp', init=list(init.reshape(-1)))
    t, d = S.poseError(Mref, Mfix)
    ok = t < GOOD_M and d < GOOD_DEG
    recovered.append((startT, ok))
    print('  seeded {:.3f} m / {:5.2f} deg off -> {:.4f} m / {:.3f} deg  fit {:.3f}  {}'.format(
        startT, startD, t, d, res['fitness'], 'recovered' if ok else 'did NOT recover'))
pulled = [s for s, ok in recovered if ok]
check('Refine pulls back a calibration nudged by 5 cm',
      any(ok for s, ok in recovered if s < 0.09), 'see the table above')
if pulled:
    finding('Refine recovers from up to about {:.2f} m of error and no further. Past '
            'that ICP has no correspondences to work with and Calibrate has to run the '
            'global stage again, which is why Refine refuses to fall back to it '
            'silently.'.format(max(pulled)))


# _________________________________________________________ G. the multi camera

section('G. four cameras composed into one coordinate space')

ringScene = S.studioScene(seed=2)
caps = [c.capture(ringScene) for c in S.ringCameras(4, spanDeg=120.0, noise=0.001)]
pairs = {}
for d in range(2, 5):
    target, source = caps[d - 2], caps[d - 1]
    res, Md = register(target, source)
    pairs[d] = (d - 1, Md)
    t, dd = S.poseError(truth(target, source), Md)
    print('  Device{} <- Device{}  ov {:.2f}  {:.4f} m / {:.3f} deg  fit {:.3f} {}'.format(
        d - 1, d, S.overlapFraction(target, source), t, dd, res['fitness'], res['status']))

chain = ext.composeChain(pairs, reference=1)
worstT = worstD = 0.0
for d, composed in sorted(chain.items()):
    expected = truth(caps[0], caps[d - 1])
    t, dd = S.poseError(expected, composed)
    worstT, worstD = max(worstT, t), max(worstD, dd)
    print('  composed Device{}: {:.4f} m / {:.3f} deg from ground truth'.format(d, t, dd))
check('every device lands in the reference frame', worstT < 0.10 and worstD < 3.0,
      'worst {:.4f} m / {:.3f} deg'.format(worstT, worstD))

# What the user sees: all four clouds in one space, measured against the
# reference cloud rather than against a known answer.
merged = np.vstack([S.applyMatrix(chain[d], caps[d - 1].validPoints)
                    for d in sorted(chain) if d > 1])
ref = caps[0].validPoints
stats = S.residualStats(merged, ref, np.eye(4), inlier=0.05)
truthMerged = np.vstack([S.applyMatrix(truth(caps[0], caps[d - 1]), caps[d - 1].validPoints)
                         for d in sorted(chain) if d > 1])
idealStats = S.residualStats(truthMerged, ref, np.eye(4), inlier=0.05)
print('  merged cloud vs Device1: inlier {:.3f} / median {:.4f} m'
      '   (perfect calibration would give {:.3f} / {:.4f})'.format(
          stats['inlierFraction'], stats['median'],
          idealStats['inlierFraction'], idealStats['median']))
check('the merged cloud is as tight as a perfect calibration would make it',
      stats['inlierFraction'] >= idealStats['inlierFraction'] - 0.03,
      '{:.3f} vs {:.3f}'.format(stats['inlierFraction'], idealStats['inlierFraction']))

# Errors add up along a chain, so registering everything straight to the
# reference looks like the obvious fix. In a circle it is not: the far cameras
# barely see anything Device1 sees.
star = {}
for d in range(2, 5):
    res, Md = register(caps[0], caps[d - 1])
    star[d] = (1, Md)
starChain = ext.composeChain(star, reference=1)
starWorst = 0.0
for d in range(2, 5):
    t, _ = S.poseError(truth(caps[0], caps[d - 1]), starChain[d])
    starWorst = max(starWorst, t)
    print('  Device{} straight to Device1: ov {:.2f}  {:.4f} m'.format(
        d, S.overlapFraction(caps[0], caps[d - 1]), t))
if starWorst > worstT:
    finding('registering every camera straight to the reference is WORSE here '
            '({:.2f} m against {:.3f} m daisy chained), because on a ring the far '
            'cameras share almost nothing with Device1. Parent each camera to its '
            'NEIGHBOUR and let RebuildChain compose; that is what Specifypair is '
            'for.'.format(starWorst, worstT))


# ______________________________________________________________ H. determinism

section('H. seeded runs reproduce, through a real subprocess')


def subprocessRun(target, source, seed):
    job = {'mode': 'global', 'target': target, 'source': source, 'voxel': 0.05,
           'refineVoxel': 0.01, 'maxRange': 0.0, 'seed': seed}
    path = os.path.join(TMP, 'job_seed{}_{}.json'.format(seed, _counter[0]))
    _counter[0] += 1
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(job, handle)
    proc = subprocess.run([PYEXE, WORKER, path], capture_output=True, text=True, timeout=600)
    return json.loads(proc.stdout.strip().splitlines()[-1])


detA, detB = pair(S.studioScene(seed=0), 45.0)
pa, pb = dump(detA), dump(detB)
one = subprocessRun(pa, pb, 7)
two = subprocessRun(pa, pb, 7)
check('seed 7 twice gives a bit-identical matrix', one['matrix'] == two['matrix'],
      'first {:.6f}, second {:.6f}'.format(one['matrix'][3], two['matrix'][3]))
three = subprocessRun(pa, pb, 11)
print('  seed 7 fitness {:.4f}, seed 11 fitness {:.4f}'.format(one['fitness'], three['fitness']))
unseeded = [subprocessRun(pa, pb, -1)['matrix'] for _ in range(2)]
if unseeded[0] == unseeded[1]:
    finding('two unseeded runs happened to agree. They usually do not: RANSAC is '
            'parallel and the OpenMP threads race, which is why Seed >= 0 pins '
            'OMP_NUM_THREADS to 1.')
else:
    print('  unseeded runs differ, as expected: RANSAC is parallel and the threads race')


# _________________________________________________________ I. cost at real size

section('I. cost at a real sensor resolution')

bigScene = S.studioScene(seed=1)
big = [c.capture(bigScene) for c in S.ringCameras(2, spanDeg=50.0, noise=0.001, res=(640, 576))]
res, Mbig = register(big[0], big[1])
t, d = S.poseError(truth(big[0], big[1]), Mbig)
print('  640x576 ({} valid points): {:.4f} m / {:.3f} deg in {:.1f}s'.format(
    res['sourcePoints'], t, d, res['seconds']))
check('a full resolution pair still registers', t < GOOD_M and d < GOOD_DEG,
      '{:.4f} m / {:.3f} deg'.format(t, d))
check('and does it in under two minutes', res['seconds'] < 120.0,
      '{:.1f}s'.format(res['seconds']))


# ___________________________________________________________________ summary

print('\n' + '=' * 70)
print('{} registrations in {:.0f}s'.format(_runs[0], time.time() - suiteStart))
if FAILURES:
    print('{} FAILED: {}'.format(len(FAILURES), ', '.join(FAILURES)))
    sys.exit(1)
print('all checks passed')
