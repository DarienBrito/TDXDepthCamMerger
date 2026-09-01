"""
Covers the TouchDesigner-side half of the extension: the pure numpy maths that
runs inside TD. Ingestion and registration live in worker.py and are covered by
test_worker.py.

This file must pass under ANY python 3.8+ with numpy, and importantly WITHOUT
open3d installed, because that is the situation inside TouchDesigner.

    python test_pipeline.py
"""

import os
import sys

import numpy as np

sys.stdout.reconfigure(encoding='utf-8')

# The extension lives in src/, this file in tests/.
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, 'src'))

import extTDAzureMerger as ext

FAILURES = []


def check(name, condition, detail=''):
    if not condition:
        FAILURES.append(name)
    print('  [{}] {}{}'.format('PASS' if condition else 'FAIL', name,
                               ('  -> ' + detail) if detail else ''))


def rigid(deg, axis, t):
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    th = np.radians(deg)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]], dtype=np.float64)
    M = np.eye(4)
    M[:3, :3] = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)
    M[:3, 3] = t
    return M


print('\nthe extension must import with no open3d present')
check('open3d not imported by the extension', 'open3d' not in sys.modules,
      'the TD side must never import it; that crashes TouchDesigner')
check('module exposes composeChain', hasattr(ext, 'composeChain'))
check('class has no open3d loader', not hasattr(ext.extTDAzureMerger, 'open3d'))
for name in ('Calibrate', 'Refine', 'RebuildChain', 'ResetCalibration', 'CheckWorker'):
    check('public method {}'.format(name), hasattr(ext.extTDAzureMerger, name))


print('\ncomposeChain')

M2 = rigid(20, [0, 1, 0], [0.5, 0.0, 0.1])
M3 = rigid(-15, [0, 1, 0], [0.3, 0.05, -0.2])
M4 = rigid(35, [1, 0, 0], [-0.1, 0.2, 0.4])

chain = ext.composeChain({2: (1, M2), 3: (2, M3), 4: (3, M4)}, reference=1)
check('reference stays identity', np.allclose(chain[1], np.eye(4)))
check('single hop is the pairwise matrix', np.allclose(chain[2], M2))
check('two hops compose parent-left', np.allclose(chain[3], M2 @ M3),
      'max diff {:.2e}'.format(np.abs(chain[3] - M2 @ M3).max()))
check('three hops compose parent-left', np.allclose(chain[4], M2 @ M3 @ M4))
check('order actually matters', not np.allclose(M2 @ M3, M3 @ M2),
      'if these matched, the ordering tests would prove nothing')

p3 = np.array([0.4, 0.9, -0.2, 1.0])
check('a point maps through the chain', np.allclose(chain[3] @ p3, M2 @ (M3 @ p3)))

star = ext.composeChain({2: (1, M2), 3: (1, M3), 4: (1, M4)}, reference=1)
check('star topology needs no composition', np.allclose(star[3], M3))

nonRoot = ext.composeChain({1: (2, np.linalg.inv(M2)), 3: (2, M3)}, reference=2)
check('reference can be a device other than 1', np.allclose(nonRoot[1], np.linalg.inv(M2)))

deep = ext.composeChain({2: (1, M2), 3: (2, M3), 4: (3, M4), 5: (4, M2), 6: (5, M3)}, 1)
check('six device chain composes', np.allclose(deep[6], M2 @ M3 @ M4 @ M2 @ M3))

try:
    ext.composeChain({2: (3, M2), 3: (2, M3)}, reference=1)
    check('cycle raises', False, 'no exception')
except ValueError as err:
    check('cycle raises', 'loops back' in str(err), str(err))

try:
    ext.composeChain({3: (2, M3)}, reference=1)
    check('orphaned device raises', False, 'no exception')
except ValueError as err:
    check('orphaned device raises', 'no calibration' in str(err), str(err))


print('\nmatrix helpers')

flat = list(M2.reshape(-1))
check('matrixFromValues round trips row major', np.allclose(ext.matrixFromValues(flat), M2))
# writeMatrix serialises cells as repr(float(v)). The float() cast is load
# bearing: under numpy 2.x repr(np.float64(0.93)) is 'np.float64(0.93)', which
# does not parse back. This asserts the exact round trip the component uses.
cells = [repr(float(v)) for v in flat]
check('table cells round trip', np.allclose(ext.matrixFromValues(cells), M2))
check('cells are plain numbers, not numpy reprs',
      not any(c.startswith('np.') for c in cells), cells[0])
check('full precision survives the round trip',
      np.array_equal(ext.matrixFromValues(cells), M2),
      'exact equality, not just allclose')
try:
    ext.matrixFromValues(flat[:15])
    check('short matrix raises', False, 'no exception')
except ValueError as err:
    check('short matrix raises', 'got 15' in str(err), str(err))
try:
    ext.matrixFromValues(['x'] * 16)
    check('non numeric raises', False, 'no exception')
except ValueError:
    check('non numeric raises', True)

check('looksRigid accepts rotation+translation', ext.looksRigid(M2))
check('looksRigid rejects a scaled matrix', not ext.looksRigid(M2 * 2.0))
bad = M2.copy()
bad[3] = [0.1, 0, 0, 1]
check('looksRigid rejects a broken bottom row', not ext.looksRigid(bad))


print('\ncalibration table schema')
check('23 columns per row (7 metadata + 16 matrix)', len(ext.CALIBRATION_COLUMNS) == 7 + 16,
      str(len(ext.CALIBRATION_COLUMNS)))
check('matrix columns are row major m00..m33',
      ext.MATRIX_COLUMNS[:5] == ['m00', 'm01', 'm02', 'm03', 'm10'])
check('device prefix is Device', ext.DEVICE_PREFIX == 'Device')


print('\n' + '=' * 60)
if FAILURES:
    print('{} FAILED: {}'.format(len(FAILURES), ', '.join(FAILURES)))
    sys.exit(1)
print('all checks passed')
