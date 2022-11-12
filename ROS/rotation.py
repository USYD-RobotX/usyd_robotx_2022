from scipy.spatial.transform import Rotation as R


r = R.from_euler('yzx', [12, 60, 0], degrees=True).as_euler('xyz', degrees=True)
print(r)


r = R.from_euler('zyx', [0, 12, 0], degrees=True).as_euler('xyz', degrees=True)
print(r)