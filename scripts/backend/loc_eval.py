# 定位轨迹在 map 系里的误差。用法：loc_eval.py <gt.tum> <建图 opt.tum> <定位轨迹.tum>...
# 先用 Umeyama 把建图轨迹对齐到真值，再用同一个变换评估定位轨迹，所以误差里也包含地图本身的误差。
import numpy as np, sys
# usage: gt map_opt.tum loc_traj.tum...   error of loc poses in the map frame, map->gt fitted on the mapping run
def umeyama(A,B):
    ma,mb=A.mean(0),B.mean(0); H=(A-ma).T@(B-mb); U,S,Vt=np.linalg.svd(H); D=np.eye(3); D[2,2]=np.sign(np.linalg.det(Vt.T@U.T)); R=Vt.T@D@U.T; return R, mb-R@ma
gt=np.loadtxt(sys.argv[1])
def assoc(est):
    idx=np.clip(np.searchsorted(gt[:,0],est[:,0]),1,len(gt)-1); prev=idx-1
    use=np.where(np.abs(gt[prev,0]-est[:,0])<np.abs(gt[idx,0]-est[:,0]),prev,idx); ok=np.abs(gt[use,0]-est[:,0])<0.02
    return est[ok], gt[use[ok]]
e,g=assoc(np.loadtxt(sys.argv[2])); R,t=umeyama(e[:,1:4],g[:,1:4])
print("map run fit rmse %.3f"%np.sqrt((np.linalg.norm((R@e[:,1:4].T).T+t-g[:,1:4],axis=1)**2).mean()))
for f in sys.argv[3:]:
    e,g=assoc(np.loadtxt(f)); err=np.linalg.norm((R@e[:,1:4].T).T+t-g[:,1:4],axis=1); tt=e[:,0]-gt[0,0]
    print("%s: n %d rmse %.3f median %.3f p95 %.3f max %.3f"%(f,len(e),np.sqrt((err**2).mean()),np.median(err),np.percentile(err,95),err.max()))
    print("  30s bins:", " ".join("%d:%.2f"%(b,err[(tt>=b)&(tt<b+30)].mean()) for b in range(0,630,30) if ((tt>=b)&(tt<b+30)).any()))
