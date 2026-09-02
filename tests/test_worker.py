"""
Drive worker.py exactly as the TouchDesigner extension will: as a subprocess,
over JSON, with .npy clouds on disk.

    python test_worker.py <python-with-open3d>
"""

import json
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.stdout.reconfigure(encoding='utf-8')

HERE = os.path.dirname(os.path.abspath(__file__))
WORKER = os.path.join(HERE, os.pardir, 'src', 'worker.py')
PYEXE = sys.argv[1] if len(sys.argv) > 1 else sys.executable

FAILURES = []


def check(name, ok, detail=''):
    if not ok:
        FAILURES.append(name)
    print('  [{}] {}{}'.format('PASS' if ok else 'FAIL', name,
                               ('  -> ' + detail) if detail else ''))


def rigid(deg, axis, t):
    axis = np.asarray(axis, float)
    axis /= np.linalg.norm(axis)
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
    sphere = v * .30 + [1.00, .35, .50]
    b = rng.uniform(-.2, .2, (6000, 3))
    b[np.arange(6000), rng.integers(0, 3, 6000)] = .2 * rng.choice([-1., 1.], 6000)
    return np.vstack([floor, sphere, b + [-.80, .20, -.60]])


def call(job=None, probe=False):
    cmd = [PYEXE, WORKER, '--probe'] if probe else [PYEXE, WORKER, job]
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    try:
        return json.loads(p.stdout.strip().splitlines()[-1]), p
    except Exception:
        return {'ok': False, 'error': 'unparseable', 'stdout': p.stdout,
                'stderr': p.stderr[-500:]}, p


print('worker :', WORKER)
print('python :', PYEXE)

print('\nprobe')
info, _ = call(probe=True)
check('probe reports ok', info.get('ok'), json.dumps(info))
print('       python {}  numpy {}  open3d {}'.format(
    info.get('python'), info.get('numpy'), info.get('open3d')))

tmp = tempfile.mkdtemp(prefix='tdxmerger_')
print('\ntemp   :', tmp)

TRUE = rigid(25, [.1, 1, .05], [.40, .03, -.30])
full = scene()
rng = np.random.default_rng(7)
tgt = full[full[:, 0] > -1.10] + rng.normal(scale=.003, size=full[full[:, 0] > -1.10].shape)
src = full[full[:, 0] < 1.10] + rng.normal(scale=.003, size=full[full[:, 0] < 1.10].shape)
tgt = (TRUE[:3, :3] @ tgt.T).T + TRUE[:3, 3]

# Pad with invalid pixels the way a real depth camera does: exact zeros.
tgtPad = np.vstack([tgt, np.zeros((20000, 3))])
srcPad = np.vstack([src, np.zeros((20000, 3))])
np.save(os.path.join(tmp, 'tgt.npy'), tgtPad)
np.save(os.path.join(tmp, 'src.npy'), srcPad)
# Too few points to pass, so the rough match grades FAIL. Moving a cloud far
# away does not work here: the rough match needs no starting guess, so it finds
# the right answer anyway.
np.save(os.path.join(tmp, 'far.npy'),
        np.random.default_rng(11).uniform(-1.5, 1.5, (60, 3)))
np.save(os.path.join(tmp, 'tgtcol.npy'), np.tile([.5, .5, .5], (len(tgtPad), 1)))
np.save(os.path.join(tmp, 'srccol.npy'), np.tile([.5, .5, .5], (len(srcPad), 1)))

base = {'target': os.path.join(tmp, 'tgt.npy'), 'source': os.path.join(tmp, 'src.npy'),
        'voxel': 0.05, 'refineVoxel': 0.01, 'maxRange': 0.0, 'seed': 3,
        'result': os.path.join(tmp, 'out.json')}


def job(name, extra):
    d = dict(base)
    d.update(extra)
    path = os.path.join(tmp, name + '.json')
    with open(path, 'w') as h:
        json.dump(d, h)
    return path


print('\nglobal registration')
res, proc = call(job('g', {'mode': 'global'}))
check('worker exits 0', proc.returncode == 0, 'rc={} err={}'.format(proc.returncode, proc.stderr[-300:]))
check('result ok', res.get('ok'), json.dumps(res)[:300])
if res.get('ok'):
    M = np.array(res['matrix']).reshape(4, 4)
    t, d = poseError(TRUE, M)
    print('       err {:.4f} m / {:.3f} deg | fitness {:.3f} | {} pts kept of {}'
          .format(t, d, res['fitness'], res['sourcePoints'], len(srcPad)))
    check('RANSAC finds the right basin', t < .15 and d < 8, '{:.4f} m / {:.3f} deg'.format(t, d))
    check('invalid zero rows were filtered', res['sourcePoints'] == len(src),
          '{} vs {}'.format(res['sourcePoints'], len(src)))
    check('result file written', os.path.isfile(base['result']))
    coarse = M

print('\nICP refine, seeded from the global result')
res2, proc2 = call(job('i', {'mode': 'icp', 'init': list(coarse.reshape(-1))}))
check('result ok', res2.get('ok'), json.dumps(res2)[:300])
if res2.get('ok'):
    M2 = np.array(res2['matrix']).reshape(4, 4)
    t2, d2 = poseError(TRUE, M2)
    print('       err {:.4f} m / {:.3f} deg | fitness {:.3f} | rmse {:.5f}'
          .format(t2, d2, res2['fitness'], res2['rmse']))
    check('ICP recovers the true transform', t2 < .02 and d2 < 1.0,
          '{:.4f} m / {:.3f} deg'.format(t2, d2))
    check('ICP improves on RANSAC', t2 < t, '{:.4f} -> {:.4f}'.format(t, t2))
    check('graded OK', res2['status'] == 'OK', res2['status'])

print('\ncoloured ICP')
res3, _ = call(job('c', {'mode': 'icp', 'colored': True, 'init': list(coarse.reshape(-1)),
                         'targetColors': os.path.join(tmp, 'tgtcol.npy'),
                         'sourceColors': os.path.join(tmp, 'srccol.npy')}))
check('coloured ICP runs', res3.get('ok'), json.dumps(res3)[:200])

print('\nglobalThenIcp: both stages in one process')
res7, proc7 = call(job('gi', {'mode': 'globalThenIcp'}))
check('result ok', res7.get('ok'), json.dumps(res7)[:300])
if res7.get('ok'):
    M7 = np.array(res7['matrix']).reshape(4, 4)
    t7, d7 = poseError(TRUE, M7)
    print('       err {:.4f} m / {:.3f} deg | fitness {:.3f} | stage {}'
          .format(t7, d7, res7['fitness'], res7['stage']))
    check('stage is globalThenIcp', res7.get('stage') == 'globalThenIcp', str(res7.get('stage')))
    check('carries the global stage numbers', isinstance(res7.get('global'), dict)
          and res7['global'].get('stage') == 'global', json.dumps(res7.get('global'))[:120])
    check('chained lands where the two-step run does', t7 < .02 and d7 < 1.0,
          '{:.4f} m / {:.3f} deg'.format(t7, d7))
    check('chained beats its own global stage', res7['global']['rmse'] > res7['rmse'],
          'global rmse {:.4f} -> icp rmse {:.4f}'.format(res7['global']['rmse'], res7['rmse']))

print('\nglobalThenIcp does not refine a failed global')
res8, _ = call(job('gifail', {'mode': 'globalThenIcp',
                              'source': os.path.join(tmp, 'far.npy')}))
check('failed global comes back as global, unrefined',
      (not res8.get('ok')) or (res8.get('stage') == 'global'),
      '{} / {}'.format(res8.get('stage'), str(res8.get('error'))[:60]))

print('\nconsensus: repeated global runs, and whether they agreed')
res9, _ = call(job('cons', {'mode': 'global', 'seed': -1}))
check('an unseeded global run reports consensus', 'consensus' in res9,
      json.dumps({k: v for k, v in res9.items() if k != 'matrix'})[:200])
check('and says whether the runs agreed', isinstance(res9.get('agreed'), bool),
      str(res9.get('agreed')))
if res9.get('ok'):
    # Never gate on ONE unseeded run. The consensus check doubts about a quarter
    # of CORRECT answers by design, because those are pairs RANSAC solved
    # unstably, so `agreed is True` on a single trial fails roughly once in four
    # and says nothing when it passes. Three trials, at least one agreement: a
    # scene this well overlapped cannot disagree with itself every time.
    spreads = [(res9['consensus'], res9['agreed'])]
    for extra in range(2):
        more, _ = call(job('cons{}'.format(extra), {'mode': 'global', 'seed': -1}))
        if more.get('ok'):
            spreads.append((more['consensus'], more['agreed']))
    print('       limit {:.4f}, spreads {}'.format(res9['consensusLimit'],
          ', '.join('{:.4f}{}'.format(v, '' if ok else ' (WARN)') for v, ok in spreads)))
    check('a well overlapped pair agrees with itself at least sometimes',
          any(ok for _, ok in spreads),
          '{} of {} runs agreed'.format(sum(1 for _, ok in spreads if ok), len(spreads)))

# A seeded run comes out the same every time, so asking four of them whether
# they agree proves nothing. The worker must not claim it checked.
res10, _ = call(job('consseed', {'mode': 'global', 'seed': 5}))
check('a seeded run reports no consensus', 'consensus' not in res10,
      str(res10.get('consensus')))
res11, _ = call(job('consoff', {'mode': 'global', 'seed': -1, 'consensusRuns': 1}))
check('consensusRuns 1 disables the check', 'consensus' not in res11,
      str(res11.get('consensus')))

# globalThenIcp has to carry the disagreement up: the refine polishes whatever
# it is handed, so its own score cannot say the rough match started wrong.
# The seed is cleared here, because a seeded run has no consensus to carry.
res12, _ = call(job('consgi', {'mode': 'globalThenIcp', 'seed': -1}))
if res12.get('ok'):
    check('globalThenIcp carries consensus up from the global stage',
          'consensus' in res12 and 'consensus' in res12.get('global', {}),
          json.dumps({k: v for k, v in res12.items() if k != 'matrix'})[:200])
    check('and the top level agrees with its global stage',
          res12.get('agreed') == res12['global'].get('agreed'),
          '{} vs {}'.format(res12.get('agreed'), res12['global'].get('agreed')))

print('\nfailure paths report cleanly instead of hanging')
res4, proc4 = call(job('bad', {'mode': 'global', 'target': os.path.join(tmp, 'nope.npy')}))
check('missing file -> ok:false', res4.get('ok') is False, str(res4.get('error'))[:90])
check('missing file -> exit 1', proc4.returncode == 1, str(proc4.returncode))

np.save(os.path.join(tmp, 'empty.npy'), np.zeros((500, 3)))
res5, _ = call(job('empty', {'mode': 'global', 'source': os.path.join(tmp, 'empty.npy')}))
check('all-invalid cloud -> ok:false', res5.get('ok') is False, str(res5.get('error'))[:90])
check('error names the cause', 'no valid points' in str(res5.get('error')), str(res5.get('error'))[:90])

res6, _ = call(job('nocol', {'mode': 'icp', 'colored': True, 'init': list(coarse.reshape(-1))}))
check('coloured ICP without colours -> ok:false', res6.get('ok') is False, str(res6.get('error'))[:90])

# A custom source can point at any TOP, so the colour image can hold a
# different number of pixels than the cloud. The worker has to say so plainly.
np.save(os.path.join(tmp, 'bigcol.npy'), np.tile([.5, .5, .5], (len(tgtPad) * 2, 1)))
res8, _ = call(job('miscol', {'mode': 'icp', 'colored': True,
                              'init': list(coarse.reshape(-1)),
                              'targetColors': os.path.join(tmp, 'bigcol.npy'),
                              'sourceColors': os.path.join(tmp, 'srccol.npy')}))
check('mismatched colour resolution -> ok:false', res8.get('ok') is False,
      str(res8.get('error'))[:90])
check('error says which cloud and both counts',
      'target colour image' in str(res8.get('error'))
      and str(len(tgtPad) * 2) in str(res8.get('error')),
      str(res8.get('error'))[:120])

print('\n' + '=' * 60)
if FAILURES:
    print('{} FAILED: {}'.format(len(FAILURES), ', '.join(FAILURES)))
    sys.exit(1)
print('all checks passed')
