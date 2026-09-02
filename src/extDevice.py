"""
	Small helpers that live inside one camera's COMP.

	Registration does not use any of this.
"""


class extDevice:
	"""Helpers for one camera."""

	# The order the Kinect TOP lists its lens values in.
	INTRINSICS = ['cx', 'cy', 'fx', 'fy', 'k1', 'k2', 'k3', 'k4', 'k5', 'k6',
		'codx', 'cody', 'p2', 'p1']

	def __init__(self, ownerComp):
		self.ownerComp = ownerComp

	def cameraTop(self):
		"""The camera TOP this device reads from, found through its select TOPs."""
		for name in ('in_pointcloud', 'in_color'):
			select = self.ownerComp.op(name)
			if select is not None and hasattr(select.par, 'top'):
				return select.par.top.eval()
		return None

	def GetIntrinsics(self):
		"""
		Lens values for the depth and colour cameras as a dict, or None.

		Only the Kinect reports them. Anything else prints a note and returns
		None, so do not rely on this.
		"""
		camera = self.cameraTop()
		if camera is None:
			print('[TDXMerger] {}: no camera TOP resolved'.format(self.ownerComp.path))
			return None

		depth = getattr(camera, 'depthCameraIntrinsics', None)
		color = getattr(camera, 'colorCameraIntrinsics', None)
		if depth is None or color is None:
			print('[TDXMerger] {} ({}) does not publish camera intrinsics'.format(
				camera.path, camera.type))
			return None

		return {name: {'depth': depth[i], 'color': color[i]}
			for i, name in enumerate(self.INTRINSICS)}
