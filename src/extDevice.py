"""
	Per device helpers for TDXDepthCamMerger.

	Nothing here is needed for registration; the merge works without it.
"""


class extDevice:
	"""Helpers local to one camera's COMP."""

	# Order matches the Kinect Azure TOP's intrinsics tuples.
	INTRINSICS = ['cx', 'cy', 'fx', 'fy', 'k1', 'k2', 'k3', 'k4', 'k5', 'k6',
		'codx', 'cody', 'p2', 'p1']

	def __init__(self, ownerComp):
		self.ownerComp = ownerComp

	def cameraTop(self):
		"""
		The camera TOP this device reads from. Resolved through whichever select
		TOP is present, so it survives the device type changing underneath.
		"""
		for name in ('in_pointcloud', 'in_color'):
			select = self.ownerComp.op(name)
			if select is not None and hasattr(select.par, 'top'):
				return select.par.top.eval()
		return None

	def GetIntrinsics(self):
		"""
		Depth and colour camera intrinsics as a dict, or None.

		Only the Kinect Azure TOP publishes these. The Orbbec TOP does not, so
		this reports and returns None there rather than raising, and no caller
		should depend on it.
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
