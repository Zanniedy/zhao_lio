# 用真值核对每个被接受的回环。用法：loop_check.py <gt.tum> <地图包/keyframes.txt> <loops.txt> <LIO 轨迹.tum>
# loops.txt：grep accepted <回放日志>。每行打印回环测量、LIO、优化后三者的相对位姿误差 [dx dy dz | 旋转向量(度)]。
import numpy as np, re, sys
from scipy.spatial.transform import Rotation as Rot
gt=np.loadtxt(sys.argv[1]); kf=np.loadtxt(sys.argv[2]); lio=np.loadtxt(sys.argv[4])
def T(row): M=np.eye(4); M[:3,:3]=Rot.from_quat(row[4:8]).as_matrix(); M[:3,3]=row[1:4]; return M
def at(a,t): return T(a[np.argmin(np.abs(a[:,0]-t))])
def kfT(i): r=kf[i]; M=np.eye(4); M[:3,:3]=Rot.from_quat(r[6:10]).as_matrix(); M[:3,3]=r[3:6]; return M
for line in open(sys.argv[3]):
    v=re.search(r"loop (\d+)-(\d+).*z (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+), t (\S+) (\S+)",line).groups(); v=list(map(float,v))
    a,b=int(v[0]),int(v[1]); Z=np.eye(4); Z[:3,:3]=Rot.from_quat(v[5:9]).as_matrix(); Z[:3,3]=v[2:5]
    G=np.linalg.inv(at(gt,v[9]))@at(gt,v[10]); L=np.linalg.inv(at(lio,v[9]))@at(lio,v[10]); O=np.linalg.inv(kfT(a))@kfT(b)
    f=lambda M: "[%+.2f %+.2f %+.2f | %+.2f %+.2f %+.2f]"%(*(M[:3,3]-G[:3,3]), *np.degrees(Rot.from_matrix(G[:3,:3].T@M[:3,:3]).as_rotvec()))
    print("%3d-%3d loopZ %s  lio %s  opt %s"%(a,b,f(Z),f(L),f(O)))
