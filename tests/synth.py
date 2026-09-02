"""
Made up depth camera clouds for the registration tests.

Shapes are traced from a virtual camera, so the clouds behave like real ones:
things hide behind other things, surfaces are only seen from one side, points
thin out with distance, steep angles return nothing, noise grows with depth,
and a pixel with no return is an exact (0, 0, 0) row, which is what the worker
filters out.

Every capture is in its OWN camera's space, with the camera at the origin, the
same as a real point cloud TOP gives. The camera's place in the world is kept
alongside as T, so the right answer for putting source onto target is
inv(T_target) @ T_source.

numpy only, no open3d, no scipy.
"""

import numpy as np

__all__ = ['Plane', 'Sphere', 'Box', 'Scene', 'DepthCamera', 'Capture',
           'Solid', 'Checker', 'Blobs',
           'lookAt', 'ringCameras', 'studioScene', 'symmetricRoom', 'planeScene',
           'texturedPlane', 'poseError', 'rigid', 'applyMatrix', 'worldPoints',
           'nearestDistances', 'residualStats', 'overlapFraction']


# ______________________________________________________________________ maths


def rigid(deg, axis, t):
    """A turn of deg degrees about axis, plus a move by t, as a 4x4."""
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


def poseError(a, b):
    """How far apart two 4x4 transforms are, in metres and degrees."""
    d = np.linalg.inv(np.asarray(a, float)) @ np.asarray(b, float)
    trace = np.clip((np.trace(d[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.linalg.norm(d[:3, 3])), float(np.degrees(np.arccos(trace)))


def applyMatrix(M, points):
    M = np.asarray(M, dtype=np.float64).reshape(4, 4)
    points = np.asarray(points, dtype=np.float64)
    return points @ M[:3, :3].T + M[:3, 3]


def lookAt(position, target, up=(0.0, 1.0, 0.0)):
    """
    Where the camera sits in the world, as a 4x4. In its own space the camera
    looks along +z with +x right and +y up, and sits at the origin of the cloud
    it makes.
    """
    position = np.asarray(position, dtype=np.float64)
    forward = np.asarray(target, dtype=np.float64) - position
    forward = forward / np.linalg.norm(forward)
    right = np.cross(np.asarray(up, dtype=np.float64), forward)
    norm = np.linalg.norm(right)
    if norm < 1e-9:                       # looking straight up or down
        right = np.cross([1.0, 0.0, 0.0], forward)
        norm = np.linalg.norm(right)
    right = right / norm
    trueUp = np.cross(forward, right)
    T = np.eye(4)
    T[:3, 0] = right
    T[:3, 1] = trueUp
    T[:3, 2] = forward
    T[:3, 3] = position
    return T


# ___________________________________________________________________ textures


class Solid:
    def __init__(self, color=(0.55, 0.55, 0.58)):
        self.color = np.asarray(color, dtype=np.float64)

    def sample(self, u, v):
        return np.tile(self.color, (len(u), 1))


class Checker:
    """
    A repeating pattern, on purpose. Coloured ICP locks onto the wrong square
    here, so use this for shape tests, not for colour tests.
    """

    def __init__(self, size=0.5, color=(0.55, 0.55, 0.58), colorB=(0.25, 0.25, 0.28)):
        self.size = float(size)
        self.color = np.asarray(color, dtype=np.float64)
        self.colorB = np.asarray(colorB, dtype=np.float64)

    def sample(self, u, v):
        odd = (np.floor(u / self.size).astype(np.int64)
               + np.floor(v / self.size).astype(np.int64)) % 2
        out = np.tile(self.color, (len(u), 1))
        out[odd == 1] = self.colorB
        return out


class Blobs:
    """
    Random patches that never repeat, so any colour match is unique. This is
    what a real patterned surface gives coloured ICP.
    """

    def __init__(self, seed=0, count=90, extent=5.0):
        rng = np.random.default_rng(seed)
        self.seeds = rng.uniform(-extent, extent, (int(count), 2))
        self.colors = rng.uniform(0.05, 0.95, (int(count), 3))

    def sample(self, u, v):
        q = np.stack([u, v], axis=1)
        d2 = ((q[:, None, :] - self.seeds[None, :, :]) ** 2).sum(axis=2)
        return self.colors[d2.argmin(axis=1)]


# _________________________________________________________________ primitives


class Plane:
    """Finite rectangle."""

    def __init__(self, point, normal, tangent, halfU, halfV, texture=None):
        self.point = np.asarray(point, dtype=np.float64)
        n = np.asarray(normal, dtype=np.float64)
        self.normal = n / np.linalg.norm(n)
        u = np.asarray(tangent, dtype=np.float64)
        u = u - self.normal * (u @ self.normal)
        self.u = u / np.linalg.norm(u)
        self.v = np.cross(self.normal, self.u)
        self.halfU = float(halfU)
        self.halfV = float(halfV)
        self.texture = texture or Solid()

    def intersect(self, origin, dirs):
        denom = dirs @ self.normal
        with np.errstate(divide='ignore', invalid='ignore'):
            t = ((self.point - origin) @ self.normal) / denom
        hit = np.isfinite(t) & (t > 1e-4) & (np.abs(denom) > 1e-9)
        p = origin + dirs * t[:, None]
        rel = p - self.point
        du = rel @ self.u
        dv = rel @ self.v
        hit &= (np.abs(du) <= self.halfU) & (np.abs(dv) <= self.halfV)
        t = np.where(hit, t, np.inf)
        # Turn the surface direction back towards the ray, so a surface is only
        # ever seen from the side facing the camera.
        normals = np.tile(self.normal, (len(dirs), 1))
        normals[denom > 0] *= -1.0
        return t, normals, self.texture.sample(du, dv)


class Sphere:
    def __init__(self, center, radius, texture=None):
        self.center = np.asarray(center, dtype=np.float64)
        self.radius = float(radius)
        self.texture = texture or Solid((0.70, 0.35, 0.30))

    def intersect(self, origin, dirs):
        oc = origin - self.center
        b = dirs @ oc
        c = float(oc @ oc) - self.radius ** 2
        disc = b * b - c
        hit = disc > 0.0
        root = np.sqrt(np.where(hit, disc, 0.0))
        t = -b - root
        behind = t <= 1e-4
        t = np.where(behind, -b + root, t)           # camera inside the sphere
        hit &= t > 1e-4
        t = np.where(hit, t, np.inf)
        p = origin + dirs * np.where(np.isfinite(t), t, 0.0)[:, None]
        normals = (p - self.center) / self.radius
        # Angles around the sphere, so a pattern sits still on the curve.
        u = np.arctan2(normals[:, 2], normals[:, 0]) * self.radius
        v = np.arcsin(np.clip(normals[:, 1], -1.0, 1.0)) * self.radius
        return t, normals, self.texture.sample(u, v)


class Box:
    def __init__(self, center, size, yawDeg=0.0, pitchDeg=0.0, texture=None):
        self.center = np.asarray(center, dtype=np.float64)
        self.half = np.asarray(size, dtype=np.float64) / 2.0
        R = rigid(yawDeg, [0, 1, 0], [0, 0, 0])[:3, :3] @ \
            rigid(pitchDeg, [1, 0, 0], [0, 0, 0])[:3, :3]
        self.R = R
        self.texture = texture or Solid((0.45, 0.62, 0.40))

    def intersect(self, origin, dirs):
        o = (origin - self.center) @ self.R          # world -> local
        d = dirs @ self.R
        with np.errstate(divide='ignore', invalid='ignore'):
            inv = 1.0 / d
            t1 = (-self.half - o) * inv
            t2 = (self.half - o) * inv
        lo = np.minimum(t1, t2)
        hi = np.maximum(t1, t2)
        tmin = np.nanmax(np.where(np.isnan(lo), -np.inf, lo), axis=1)
        tmax = np.nanmin(np.where(np.isnan(hi), np.inf, hi), axis=1)
        hit = (tmax >= np.maximum(tmin, 1e-4))
        t = np.where(tmin > 1e-4, tmin, tmax)
        t = np.where(hit & (t > 1e-4), t, np.inf)
        axis = np.argmax(np.where(np.isnan(lo), -np.inf, lo), axis=1)
        rows = np.arange(len(dirs))
        localN = np.zeros((len(dirs), 3))
        localN[rows, axis] = -np.sign(d[rows, axis])
        normals = localN @ self.R.T
        other = [(1, 2), (0, 2), (0, 1)]
        p = o + d * np.where(np.isfinite(t), t, 0.0)[:, None]
        ua = np.array([other[a][0] for a in axis])
        va = np.array([other[a][1] for a in axis])
        return t, normals, self.texture.sample(p[rows, ua], p[rows, va])


class Scene:
    def __init__(self, primitives):
        self.primitives = list(primitives)

    def raycast(self, origin, dirs):
        best = np.full(len(dirs), np.inf)
        normals = np.zeros((len(dirs), 3))
        colors = np.zeros((len(dirs), 3))
        for prim in self.primitives:
            t, n, c = prim.intersect(origin, dirs)
            closer = t < best
            if not closer.any():
                continue
            best = np.where(closer, t, best)
            normals[closer] = n[closer]
            colors[closer] = c[closer]
        return best, normals, colors


# ______________________________________________________________________ camera


class Capture:
    """One frame: a HxW grid of points flattened to (H*W, 3), in camera space."""

    def __init__(self, points, colors, valid, T, res):
        self.points = points
        self.colors = colors
        self.valid = valid
        self.T = T
        self.res = res

    @property
    def validPoints(self):
        return self.points[self.valid]

    def world(self):
        return applyMatrix(self.T, self.validPoints)

    def save(self, path):
        np.save(path, self.points)
        return path

    def saveColors(self, path):
        np.save(path, self.colors)
        return path


class DepthCamera:
    """
    noise is how far a reading wobbles at one metre; it grows with the square
    of the distance, the way real depth sensors do. grazingDeg throws away
    readings from surfaces seen too edge on, which is what puts the holes at
    the edges of things.
    """

    def __init__(self, position, target, up=(0.0, 1.0, 0.0), res=(240, 180),
                 fovDeg=70.0, noise=0.0015, dropout=0.005, grazingDeg=82.0,
                 maxDepth=12.0, seed=0):
        self.T = lookAt(position, target, up)
        self.res = tuple(int(v) for v in res)
        self.fovDeg = float(fovDeg)
        self.noise = float(noise)
        self.dropout = float(dropout)
        self.grazingDeg = float(grazingDeg)
        self.maxDepth = float(maxDepth)
        self.seed = int(seed)

    def rays(self):
        w, h = self.res
        fx = (w / 2.0) / np.tan(np.radians(self.fovDeg) / 2.0)
        fy = fx
        i, j = np.meshgrid(np.arange(w), np.arange(h))
        x = (i.ravel() + 0.5 - w / 2.0) / fx
        y = -(j.ravel() + 0.5 - h / 2.0) / fy
        d = np.stack([x, y, np.ones_like(x)], axis=1)
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def capture(self, scene, extra=()):
        """extra: shapes only THIS camera can see, for scenes that change."""
        rng = np.random.default_rng(self.seed)
        camDirs = self.rays()
        R = self.T[:3, :3]
        origin = self.T[:3, 3]
        worldDirs = camDirs @ R.T
        if extra:
            scene = Scene(list(scene.primitives) + list(extra))
        t, normals, colors = scene.raycast(origin, worldDirs)

        valid = np.isfinite(t) & (t < self.maxDepth)
        incidence = np.abs(np.einsum('ij,ij->i', worldDirs, normals))
        valid &= incidence > np.cos(np.radians(self.grazingDeg))
        if self.dropout > 0:
            valid &= rng.random(len(t)) >= self.dropout
        if self.noise > 0:
            depth = np.where(valid, t, 1.0)
            t = t + rng.normal(0.0, self.noise * depth ** 2)

        points = np.zeros((len(t), 3), dtype=np.float64)
        points[valid] = camDirs[valid] * t[valid, None]
        out = np.zeros((len(t), 3), dtype=np.float64)
        out[valid] = colors[valid]
        return Capture(points.astype(np.float32), out.astype(np.float32),
                       valid, self.T, self.res)


def ringCameras(count, radius=2.4, height=1.35, target=(0.0, 0.65, 0.0),
                spanDeg=360.0, startDeg=0.0, **kwargs):
    """Cameras in a circle looking inwards, the way a real rig is set up."""
    cams = []
    for k in range(count):
        step = spanDeg / count if spanDeg >= 359.9 else spanDeg / max(count - 1, 1)
        a = np.radians(startDeg + k * step)
        pos = (radius * np.sin(a), height, radius * np.cos(a))
        cams.append(DepthCamera(pos, target, seed=1000 + k, **kwargs))
    return cams


# ______________________________________________________________________ scenes


def studioScene(seed=0, clutter=6, floor=1.3):
    """
    A capture space set up the way a real one is: a small piece of floor and a
    pile of objects filling the frame, so what the cameras share is the objects
    rather than an expanse of floor.

    Three things here matter and are all deliberate:

    - The floor is small. A big one takes over the score and lets a wrong,
      floor on floor answer beat the right one. See symmetricRoom.
    - Two big objects of different shape, so turning the scene never looks the
      same as leaving it alone.
    - The clutter sits at RANDOM angles. Objects spaced evenly round a circle
      make a repeating pattern of their own.
    """
    rng = np.random.default_rng(seed)
    prims = [Plane([0, 0, 0], [0, 1, 0], [1, 0, 0], floor, floor,
                   Checker(0.35, (0.50, 0.48, 0.45), (0.34, 0.33, 0.31)))]
    prims.append(Box([0.35, 0.75, -0.25], [0.55, 1.50, 0.42],
                     yawDeg=rng.uniform(0, 180), texture=Solid((0.70, 0.40, 0.30))))
    prims.append(Box([-0.55, 0.30, 0.45], [1.10, 0.60, 0.45],
                     yawDeg=rng.uniform(0, 180), pitchDeg=15.0,
                     texture=Solid((0.30, 0.50, 0.70))))
    for k in range(int(clutter)):
        a = rng.uniform(0, 2 * np.pi)
        r = rng.uniform(0.2, 0.9)
        x, z = r * np.cos(a), r * np.sin(a)
        if k % 3 == 2:
            prims.append(Sphere([x, rng.uniform(0.35, 1.10), z],
                                rng.uniform(0.20, 0.34), Solid(rng.uniform(0.2, 0.8, 3))))
        else:
            size = rng.uniform(0.25, 0.55, 3) * [1.0, rng.uniform(1.1, 2.4), 1.0]
            prims.append(Box([x, size[1] / 2, z], size,
                             yawDeg=rng.uniform(0, 180),
                             pitchDeg=rng.uniform(-25, 25),
                             texture=Solid(rng.uniform(0.2, 0.8, 3))))
    return Scene(prims)


def symmetricRoom(seed=0, objects=5, span=4.0):
    """
    Big square floor and two walls at right angles, so the room looks the same
    turned by 90 or 180 degrees. The trap scene: the rough match is expected to
    fail here, and the point is to measure that failure, not to pass.
    """
    rng = np.random.default_rng(seed)
    prims = [
        Plane([0, 0, 0], [0, 1, 0], [1, 0, 0], span, span,
              Checker(0.5, (0.50, 0.48, 0.45), (0.34, 0.33, 0.31))),
        Plane([0, 1.6, -span], [0, 0, 1], [1, 0, 0], span, 1.6, Solid((0.60, 0.58, 0.56))),
        Plane([-span, 1.6, 0], [1, 0, 0], [0, 0, 1], span, 1.6, Solid((0.42, 0.44, 0.50))),
    ]
    for k in range(int(objects)):
        a = rng.uniform(0, 2 * np.pi)
        r = rng.uniform(0.4, 1.9)
        x, z = r * np.cos(a), r * np.sin(a)
        s = rng.uniform(0.30, 0.55)
        prims.append(Box([x, s / 2, z], [s, s * rng.uniform(0.8, 2.0), s],
                         yawDeg=rng.uniform(0, 90)))
    return Scene(prims)


def planeScene(texture=None):
    """
    One flat surface. Sliding along it or spinning on it changes nothing you
    can see in the shape, so this is where an answer can look confident and be
    completely wrong.
    """
    return Scene([Plane([0, 0, 0], [0, 1, 0], [1, 0, 0], 5.0, 5.0,
                        texture or Solid((0.85, 0.80, 0.70)))])


def texturedPlane(seed=0):
    """
    The same flat surface, painted with patches that never repeat. The shape
    cannot say how far it slid, the colours can. This is what Use coloured ICP
    is for.
    """
    return Scene([Plane([0, 0, 0], [0, 1, 0], [1, 0, 0], 5.0, 5.0,
                        Blobs(seed=seed, count=140, extent=5.0))])


# _____________________________________________________________________ metrics


def worldPoints(capture, matrix=None):
    pts = capture.validPoints.astype(np.float64)
    return applyMatrix(matrix if matrix is not None else capture.T, pts)


def nearestDistances(query, reference, chunk=512):
    """
    Distance from each point to the nearest one in the other cloud, worked out
    the plain way in batches. Thin the clouds first: 5k points against 40k
    takes under a tenth of a second and is plenty.
    """
    query = np.asarray(query, dtype=np.float32)
    reference = np.asarray(reference, dtype=np.float32)
    out = np.empty(len(query), dtype=np.float32)
    rr = np.einsum('ij,ij->i', reference, reference)
    for start in range(0, len(query), chunk):
        q = query[start:start + chunk]
        d2 = rr[None, :] - 2.0 * (q @ reference.T) + \
            np.einsum('ij,ij->i', q, q)[:, None]
        out[start:start + chunk] = np.sqrt(np.maximum(d2.min(axis=1), 0.0))
    return out


def _subsample(points, count, seed=0):
    if len(points) <= count:
        return points
    idx = np.random.default_rng(seed).choice(len(points), count, replace=False)
    return points[idx]


def residualStats(sourcePts, targetPts, matrix, inlier=0.05, samples=4000, seed=0):
    """
    How well the two clouds sit on top of each other once matrix is applied.
    This is what a user sees; the error against a known answer only counts
    because these scenes come with one.
    """
    moved = applyMatrix(matrix, _subsample(np.asarray(sourcePts), samples, seed))
    d = nearestDistances(moved, _subsample(np.asarray(targetPts), 40000, seed + 1))
    within = d < inlier
    return {
        'inlierFraction': float(within.mean()),
        'median': float(np.median(d)),
        'p90': float(np.percentile(d, 90)),
        'inlierMedian': float(np.median(d[within])) if within.any() else float('nan'),
    }


def overlapFraction(capA, capB, radius=0.05, samples=4000, seed=0):
    """How much of what A sees B sees too, as a fraction. A property of the scene."""
    a = _subsample(worldPoints(capA), samples, seed)
    b = _subsample(worldPoints(capB), 40000, seed + 1)
    return float((nearestDistances(a, b) < radius).mean())
