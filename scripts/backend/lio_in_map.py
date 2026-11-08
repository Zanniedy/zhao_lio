# 纯 LIO 基线：把 LIO 轨迹用后端第一帧可信输出的 T_map_odom 变换到 map 系，之后不再修正。
# 用法：lio_in_map.py <后端实时轨迹.tum> <LIO 轨迹.tum> <输出.tum>
import numpy as np, sys
from scipy.spatial.transform import Rotation as Rot
rt=np.loadtxt(sys.argv[1]); lio=np.loadtxt(sys.argv[2])
def T(r): M=np.eye(4); M[:3,:3]=Rot.from_quat(r[4:8]).as_matrix(); M[:3,3]=r[1:4]; return M
i=np.argmin(np.abs(lio[:,0]-rt[0,0])); A=T(rt[0])@np.linalg.inv(T(lio[i]))
out=[]
for r in lio[lio[:,0]>=rt[0,0]]:
    M=A@T(r); q=Rot.from_matrix(M[:3,:3]).as_quat(); out.append([r[0],*M[:3,3],*q])
np.savetxt(sys.argv[3],np.array(out),fmt="%.9f")
