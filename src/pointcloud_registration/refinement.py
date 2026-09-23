"""Stage-4 explicit-region local rigid refinement (INSPIRE -> FAST)."""
from __future__ import annotations
import hashlib
from typing import Any, Callable
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from .transforms import apply_transform, rotation_difference_deg, validate_rigid_transform

class RefinementError(RuntimeError): pass

def stage4_contract()->dict[str,Any]:
    return {"stage":4,"refinement_performed":True,"final_scoring_performed":False,"final_transform_selected":False,
            "full_cloud_transformed":False,"transform_direction":"INSPIRE_TO_FAST"}

def refined_candidate_id(parent_candidate_id:str)->str:
    return f"s4_ref_{hashlib.sha256(f'stage4:{parent_candidate_id}'.encode()).hexdigest()[:12]}"

def validate_level_configuration(cfg:dict[str,Any])->None:
    g=cfg.get("gicp_levels",[]);p=cfg.get("robust_point_to_plane_levels",[])
    if not g or not p: raise RefinementError("GICP and robust point-to-plane levels are both required")
    ds=[float(x["max_correspondence_distance_m"]) for x in g+p]
    if any(a<=b for a,b in zip(ds,ds[1:])): raise RefinementError("all stage-4 correspondence thresholds must be strictly decreasing")
    if ds[-1]>=.015 or max(ds)>.015: raise RefinementError("stage-4 capture/final threshold contract failed")
    if any(float(x["voxel_size_m"])<=0 or int(x["max_iterations"])<1 for x in g+p): raise RefinementError("invalid level size/count")
    if any(float(x.get("huber_k_m",0))<=0 for x in p): raise RefinementError("positive Huber scale required")
    gates=cfg.get("sanity_gates",{});limits=gates.get("maximum_step_translation_by_voxel_m",{})
    expected={"0.005":.0025,"0.004":.002,"0.003":.0015}
    if set(limits)!=set(expected) or any(float(limits[k])!=v for k,v in expected.items()):
        raise RefinementError("stage-4 per-voxel translation limits must be 5/4/3 mm -> 2.5/2.0/1.5 mm")
    if "maximum_step_translation_m" in gates:raise RefinementError("legacy uniform translation limit is forbidden")
    factor=float(gates.get("translation_backtracking_factor",0));attempts=int(gates.get("translation_backtracking_max_attempts",0))
    if not 0<factor<1 or attempts<1:raise RefinementError("invalid deterministic translation backtracking configuration")

def deterministic_sample(points:np.ndarray,count:int,seed:int)->np.ndarray:
    pts=np.asarray(points,float)
    if len(pts)<=count:return pts.copy()
    return pts[np.sort(np.random.default_rng(seed).choice(len(pts),count,False))].copy()

def _cloud(points:np.ndarray)->o3d.geometry.PointCloud:
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(points,float)))

def _plane_normal(points:np.ndarray)->np.ndarray:
    pts=np.asarray(points,float)
    if len(pts)<3 or not np.isfinite(pts).all():raise RefinementError("cannot estimate finite plane normal")
    n=np.linalg.svd(pts-np.mean(pts,axis=0),full_matrices=False)[2][-1];n/=np.linalg.norm(n)
    return -n if n[int(np.argmax(np.abs(n)))]<0 else n

def prepare_balanced_level(board:o3d.geometry.PointCloud,obj:o3d.geometry.PointCloud,voxel_size_m:float,
    sampling_cfg:dict[str,Any],normal_cfg:dict[str,Any],covariance_cfg:dict[str,Any],seed:int
    )->tuple[dict[str,o3d.geometry.PointCloud],dict[str,Any]]:
    """Prepare labeled regions; never concatenate them for neighbour search."""
    raw={"board":board.voxel_down_sample(float(voxel_size_m)),"object":obj.voxel_down_sample(float(voxel_size_m))}
    counts={k:len(v.points) for k,v in raw.items()}
    if min(counts.values())<3:raise RefinementError("semantic region too small after voxelization")
    cap=int(sampling_cfg["maximum_points_per_region"]);on=min(counts["object"],cap)
    bn=min(counts["board"],cap,max(1,int(on*float(sampling_cfg["maximum_board_to_object_ratio"]))))
    chosen={"board":deterministic_sample(np.asarray(raw["board"].points),bn,seed),
            "object":deterministic_sample(np.asarray(raw["object"].points),on,seed^0x5A5A5A5A)}
    radius=float(normal_cfg["radius_ratio"])*voxel_size_m;cr=float(covariance_cfg["radius_ratio"])*voxel_size_m
    regions={};reports=[]
    for name,pts in chosen.items():
        pc=_cloud(pts);pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius,max_nn=int(normal_cfg["max_nn"])))
        ns=np.asarray(pc.normals).copy();ref=_plane_normal(pts);lens=np.linalg.norm(ns,axis=1)
        if not np.all(np.isfinite(ns)) or np.any(lens<=1e-12):raise RefinementError(f"{name} normals degenerate")
        ns/=lens[:,None];ns[(ns@ref)<0]*=-1;pc.normals=o3d.utility.Vector3dVector(ns)
        pc.estimate_covariances(o3d.geometry.KDTreeSearchParamHybrid(radius=cr,max_nn=int(covariance_cfg["max_nn"])))
        cov=np.asarray(pc.covariances).copy()
        if cov.shape!=(len(pts),3,3) or not np.isfinite(cov).all():raise RefinementError(f"{name} covariances unavailable")
        cov+=np.eye(3)[None]*float(covariance_cfg.get("gicp_epsilon",.001))**2
        pc.covariances=o3d.utility.Matrix3dVector(cov);regions[name]=pc
        reports.append({"region":name,"point_count":len(pts),"reference_plane_normal":ref.tolist(),"normal_radius_m":radius,"covariance_radius_m":cr})
    total=sum(len(x.points) for x in regions.values())
    return regions,{"method":sampling_cfg["method"],"explicit_region_labels":True,"cross_region_correspondence_possible":False,
        "derived_directly_from_canonical_input":True,"independent_voxelization":True,"voxel_size_m":voxel_size_m,"sampling_seed":seed,"voxelized_counts":counts,
        "balanced_counts":{k:len(v.points) for k,v in regions.items()},"proportions":{k:len(v.points)/total for k,v in regions.items()},
        "normals_and_covariances":reports}

def _trimmed_mean(x:np.ndarray,f:float)->float:
    if not len(x):return float("inf")
    n=max(1,int(np.ceil(len(x)*f)));return float(np.mean(np.partition(x,n-1)[:n]))

def region_diagnostics(matrix:np.ndarray,source_board:np.ndarray,source_object:np.ndarray,target_board:np.ndarray,
    target_object:np.ndarray,trim_fraction:float,overlap_thresholds_m:list[float]|tuple[float,...],sample_limit:int=3000)->dict[str,Any]:
    rigid=validate_rigid_transform(matrix)
    if np.isscalar(overlap_thresholds_m): overlap_thresholds_m=[float(overlap_thresholds_m)]
    if not rigid["valid"]:return {"valid":False,"rigid":rigid}
    sb=deterministic_sample(source_board,sample_limit,1701);so=deterministic_sample(source_object,sample_limit,1702)
    tb=deterministic_sample(target_board,sample_limit,2701);to=deterministic_sample(target_object,sample_limit,2702)
    sn,tn=_plane_normal(sb),_plane_normal(tb);mn=matrix[:3,:3]@sn
    angle=float(np.degrees(np.arccos(np.clip(abs(mn@tn),0,1))));msb=apply_transform(sb,matrix);mso=apply_transform(so,matrix)
    pd=float(abs(np.median((msb-np.mean(tb,axis=0))@tn)));d1=cKDTree(to).query(mso,workers=1)[0];d2=cKDTree(mso).query(to,workers=1)[0]
    both=np.r_[d1,d2];overlaps={f"at_{1000*x:g}mm":.5*(float(np.mean(d1<=x))+float(np.mean(d2<=x))) for x in overlap_thresholds_m}
    return {"valid":True,"rigid":rigid,"board":{"normal_angle_deg":angle,"plane_distance_m":pd},"object":{
        "bidirectional_trimmed_distance_m":.5*(_trimmed_mean(d1,trim_fraction)+_trimmed_mean(d2,trim_fraction)),
        "bidirectional_distance_percentiles":{f"p{p}_m":float(np.percentile(both,p)) for p in (50,80,90)},
        "bidirectional_overlap_ratios":overlaps,"trim_fraction":trim_fraction}}

def transform_increment(new:np.ndarray,old:np.ndarray)->tuple[np.ndarray,float,float]:
    d=np.asarray(new,float)@np.linalg.inv(np.asarray(old,float));return d,rotation_difference_deg(np.eye(4),d),float(np.linalg.norm(d[:3,3]))

def step_translation_limit(level:dict[str,Any],gates:dict[str,Any])->float:
    key=f"{float(level['voxel_size_m']):.3f}"
    try:return float(gates["maximum_step_translation_by_voxel_m"][key])
    except (KeyError,TypeError,ValueError) as exc:raise RefinementError(f"missing translation limit for voxel size {key} m") from exc

def _scaled_increment(delta:np.ndarray,scale:float)->np.ndarray:
    """Deterministically shrink a rigid left increment, scaling rotation and translation together."""
    if not 0<scale<=1:raise RefinementError(f"invalid step scale {scale}")
    out=np.eye(4);rotvec=Rotation.from_matrix(np.asarray(delta,float)[:3,:3]).as_rotvec()
    out[:3,:3]=Rotation.from_rotvec(rotvec*scale).as_matrix();out[:3,3]=np.asarray(delta,float)[:3,3]*scale
    return out

def _skew(p:np.ndarray)->np.ndarray:return np.array([[0.,-p[2],p[1]],[p[2],0.,-p[0]],[-p[1],p[0],0.]])

def _exp_se3(x:np.ndarray)->np.ndarray:
    w,v=x[:3],x[3:];t=np.linalg.norm(w);W=_skew(w)
    if t<1e-12:R=np.eye(3)+W;V=np.eye(3)+.5*W
    else:R=np.eye(3)+np.sin(t)/t*W+(1-np.cos(t))/t**2*(W@W);V=np.eye(3)+(1-np.cos(t))/t**2*W+(t-np.sin(t))/t**3*(W@W)
    out=np.eye(4);out[:3,:3]=R;out[:3,3]=V@v;return out

def _explicit_region_iteration(source:dict[str,o3d.geometry.PointCloud],target:dict[str,o3d.geometry.PointCloud],threshold:float,
    init:np.ndarray,algorithm:str,level:dict[str,Any])->dict[str,Any]:
    blocks=[];gradients=[];reports={};cost=0.;total=0
    for name in ("board","object"):
        sp=np.asarray(source[name].points);tp=np.asarray(target[name].points);moved=apply_transform(sp,init)
        ds,ids=cKDTree(tp).query(moved,workers=1);keep=np.isfinite(ds)&(ds<=threshold);si=np.flatnonzero(keep);ti=ids[keep]
        H=np.zeros((6,6));g=np.zeros(6);weights=[]
        for i,j in zip(si,ti):
            p,q=moved[i],tp[j]
            if algorithm=="robust_point_to_plane":
                n=np.asarray(target[name].normals)[j];r=float(n@(p-q));k=float(level["huber_k_m"]);w=1. if abs(r)<=k else k/max(abs(r),1e-15)
                J=np.r_[np.cross(p,n),n];H+=w*np.outer(J,J);g+=w*J*r;cost+=w*r*r;weights.append(w)
            elif algorithm=="gicp":
                cs=np.asarray(source[name].covariances)[i];ct=np.asarray(target[name].covariances)[j];C=ct+init[:3,:3]@cs@init[:3,:3].T
                W=np.linalg.pinv(C,hermitian=True);r=p-q;J=np.c_[-_skew(p),np.eye(3)];H+=J.T@W@J;g+=J.T@W@r;cost+=float(r@W@r);weights.append(float(np.trace(W)/3))
            else:raise RefinementError(f"unsupported algorithm {algorithm}")
        n=len(si);total+=n
        if n:H/=n;g/=n
        blocks.append(H);gradients.append(g);reports[name]={"source_point_count":len(sp),"target_point_count":len(tp),"correspondence_count":n,
            "correspondence_ratio":n/max(len(sp),1),"rmse_m":float(np.sqrt(np.mean(ds[keep]**2))) if n else None,
            "robust_weight":{"minimum":float(np.min(weights)) if weights else None,"median":float(np.median(weights)) if weights else None,"maximum":float(np.max(weights)) if weights else None}}
    H=sum(blocks);g=sum(gradients);rank=int(np.linalg.matrix_rank(H,tol=1e-10));cond=float(np.linalg.cond(H))
    if not np.isfinite(H).all() or rank<3:raise RefinementError(f"joint regional normal equation degenerate (rank={rank})")
    # Point-to-plane on a planar scene is intentionally rank deficient in tangent directions;
    # the minimum-norm update preserves those unobservable components instead of inventing drift.
    step=-np.linalg.pinv(H,hermitian=True,rcond=1e-10)@g
    return {"transformation":_exp_se3(step)@init,"regions":reports,"joint_cost":cost,"joint_correspondence_count":total,
            "normal_equation_condition":cond,"normal_equation_rank":rank,
            "solver":"equal-region normalized joint SE(3) minimum-norm normal equation"}

def _overlap(m:dict[str,Any],key:str)->float:return float(m["object"]["bidirectional_overlap_ratios"][key])

def _centroid_shift(parent:np.ndarray,proposed:np.ndarray,source_object:np.ndarray,target_board:np.ndarray)->float:
    n=_plane_normal(target_board);c=np.mean(source_object,axis=0);d=apply_transform([c],proposed)[0]-apply_transform([c],parent)[0]
    return float(np.linalg.norm(d-n*(d@n)))

def run_level(source:dict[str,o3d.geometry.PointCloud],target:dict[str,o3d.geometry.PointCloud],init:np.ndarray,parent:np.ndarray,
    algorithm:str,level_index:int,level:dict[str,Any],convergence:dict[str,Any],gates:dict[str,Any],diagnostic:Callable[[np.ndarray],dict[str,Any]],
    source_object:np.ndarray|None=None,target_board:np.ndarray|None=None,runner:Callable[...,Any]=_explicit_region_iteration)->tuple[np.ndarray,dict[str,Any]]:
    threshold=float(level["max_correspondence_distance_m"]);current=np.asarray(init,float).copy();before=diagnostic(current);pm=diagnostic(parent);traj=[];stable=0;stop="maximum_iterations_reached"
    translation_limit=step_translation_limit(level,gates);backtrack_factor=float(gates["translation_backtracking_factor"])
    backtrack_max=int(gates["translation_backtracking_max_attempts"])
    for iteration in range(1,int(level["max_iterations"])+1):
        try:r=runner(source,target,threshold,current,algorithm,level)
        except Exception as exc:
            stop="numerical_failure";traj.append({"iteration":iteration,"accepted":False,"rejection_reasons":[f"numerical_failure: {type(exc).__name__}: {exc}"],
                "matrix_m":current.tolist(),"delta_matrix":np.eye(4).tolist(),"delta_rotation_deg":0.,"delta_translation_m":0.,
                "from_parent_rotation_deg":rotation_difference_deg(parent,current),"from_parent_translation_m":float(np.linalg.norm(current[:3,3]-parent[:3,3])),
                "step_translation_limit_m":translation_limit,"backtracking_performed":False,"backtracking_attempts":[],
                "final_decision_reason":"numerical_failure","regions":{}});break
        raw_proposed=np.asarray(r["transformation"],float);raw_rigid=validate_rigid_transform(raw_proposed);cm=diagnostic(current)
        for name in ("board","object"):
            rr=r["regions"][name];minimum=max(int(gates["minimum_correspondence_absolute"][name]),int(np.ceil(float(gates["minimum_correspondence_ratio"][name])*rr["source_point_count"])))
            rr["minimum_required"]=minimum
        def safety_check(candidate:np.ndarray)->dict[str,Any]:
            rigid=validate_rigid_transform(candidate);reasons=list(rigid["reasons"]) if not rigid["valid"] else []
            delta,rd,td=transform_increment(candidate,current) if rigid["valid"] else (np.eye(4),0.,0.)
            after=diagnostic(candidate) if rigid["valid"] else {"valid":False};pr=rotation_difference_deg(parent,candidate) if rigid["valid"] else float("inf")
            pt=float(np.linalg.norm(candidate[:3,3]-parent[:3,3])) if rigid["valid"] else float("inf")
            for name in ("board","object"):
                if r["regions"][name]["correspondence_count"]<r["regions"][name]["minimum_required"]:reasons.append(f"{name}_correspondence_below_minimum")
            if rd>float(gates["maximum_step_rotation_deg"]):reasons.append("step_rotation_trust_region_exceeded")
            if td>translation_limit:reasons.append("step_translation_trust_region_exceeded")
            if pr>float(gates["maximum_parent_rotation_drift_deg"]):reasons.append("parent_rotation_trust_region_exceeded")
            if pt>float(gates["maximum_parent_translation_drift_m"]):reasons.append("parent_translation_trust_region_exceeded")
            shift=None
            if after.get("valid"):
                b,pb=after["board"],pm["board"]
                if b["normal_angle_deg"]>pb["normal_angle_deg"]+float(gates["maximum_board_normal_worsening_deg"]):reasons.append("board_normal_vs_parent_worsened")
                if b["plane_distance_m"]>pb["plane_distance_m"]+float(gates["maximum_board_plane_distance_worsening_m"]):reasons.append("board_plane_vs_parent_worsened")
                if b["normal_angle_deg"]>float(gates["maximum_board_normal_angle_deg"]):reasons.append("board_normal_absolute_limit_exceeded")
                if b["plane_distance_m"]>float(gates["maximum_board_plane_distance_m"]):reasons.append("board_plane_absolute_limit_exceeded")
                ao,co,po=after["object"],cm["object"],pm["object"]
                di=ao["bidirectional_trimmed_distance_m"]<=co["bidirectional_trimmed_distance_m"]-float(gates["minimum_object_trimmed_improvement_m"])
                oi=any(_overlap(after,key)>=_overlap(cm,key)+float(gates["minimum_object_overlap_improvement"]) for key in ("at_2.5mm","at_5mm"))
                if not(di or oi):reasons.append("object_has_no_clear_improvement")
                if ao["bidirectional_trimmed_distance_m"]>po["bidirectional_trimmed_distance_m"]+float(gates["maximum_object_trimmed_worsening_m"]):reasons.append("object_trimmed_vs_parent_worsened")
                if _overlap(after,"at_5mm")<_overlap(pm,"at_5mm")-float(gates["maximum_object_overlap_drop"]):reasons.append("object_overlap5_vs_parent_dropped")
                if source_object is not None and target_board is not None:
                    shift=_centroid_shift(parent,candidate,source_object,target_board)
                    if shift>float(gates["maximum_object_centroid_inplane_shift_m"]):reasons.append("object_centroid_inplane_shift_exceeded")
            return {"accepted":not reasons,"rejection_reasons":reasons,"matrix_m":candidate.tolist(),"delta_matrix":delta.tolist(),
                "delta_rotation_deg":rd,"delta_translation_m":td,"from_parent_rotation_deg":pr,"from_parent_translation_m":pt,
                "object_centroid_inplane_shift_from_parent_m":shift,"region_metrics":after,"rigid_validation":rigid}
        raw_check=safety_check(raw_proposed);raw_delta=np.asarray(raw_check["delta_matrix"]);raw_td=float(raw_check["delta_translation_m"])
        backtracking=bool(raw_rigid["valid"] and raw_td>translation_limit)
        if backtracking:
            # Keep the first scaled candidate strictly inside the hard limit despite floating-point roundoff.
            initial_scale=min(1.,translation_limit/raw_td)*(1.-1e-12);scales=[initial_scale*backtrack_factor**i for i in range(backtrack_max)]
        else:scales=[1.]
        attempts=[];chosen=None
        for attempt_index,scale in enumerate(scales,1):
            candidate=(_scaled_increment(raw_delta,scale)@current) if backtracking else raw_proposed
            checked=safety_check(candidate);checked.update({"attempt":attempt_index,"scale_ratio":scale})
            attempts.append(checked)
            if checked["accepted"]:chosen=checked;break
        final_check=chosen or attempts[-1];accepted=chosen is not None
        if accepted:current=np.asarray(final_check["matrix_m"],float)
        reasons=list(final_check["rejection_reasons"])
        if backtracking and not accepted:reasons.append("translation_backtracking_exhausted")
        decision=("accepted_after_translation_backtracking" if backtracking else "accepted_without_backtracking") if accepted else (
            "rejected_after_translation_backtracking_exhausted" if backtracking else "rejected_by_constrained_acceptance")
        rd=float(final_check["delta_rotation_deg"]);td=float(final_check["delta_translation_m"])
        traj.append({"iteration":iteration,"accepted":accepted,"rejection_reasons":reasons,"final_decision_reason":decision,
            "matrix_m":current.tolist(),"proposed_matrix_m":final_check["matrix_m"],"raw_proposed_matrix_m":raw_proposed.tolist(),
            "raw_delta_matrix":raw_check["delta_matrix"],"raw_delta_rotation_deg":raw_check["delta_rotation_deg"],
            "raw_delta_translation_m":raw_check["delta_translation_m"],"raw_proposal_rejection_reasons":raw_check["rejection_reasons"],
            "delta_matrix":final_check["delta_matrix"],"delta_rotation_deg":rd,"delta_translation_m":td,
            "step_translation_limit_m":translation_limit,"step_scale_ratio":final_check["scale_ratio"],"backtracking_performed":backtracking,
            "backtracking_attempts":attempts if backtracking else [],"from_parent_rotation_deg":rotation_difference_deg(parent,current),
            "from_parent_translation_m":float(np.linalg.norm(current[:3,3]-parent[:3,3])),
            "proposed_from_parent_rotation_deg":final_check["from_parent_rotation_deg"],"proposed_from_parent_translation_m":final_check["from_parent_translation_m"],
            "object_centroid_inplane_shift_from_parent_m":final_check["object_centroid_inplane_shift_from_parent_m"],"regions":r["regions"],
            "joint_cost":r["joint_cost"],"normal_equation_condition":r["normal_equation_condition"],"region_metrics_after":diagnostic(current),
            "proposed_region_metrics":final_check["region_metrics"],"raw_proposed_region_metrics":raw_check["region_metrics"],
            "rigid_validation":final_check["rigid_validation"]})
        if not accepted:
            stop="low_correspondence" if any("correspondence_below" in x for x in reasons) else "update_rejected_by_constrained_acceptance";break
        converged=rd<=float(convergence["rotation_step_deg"]) and td<=float(convergence["translation_step_m"]);stable=stable+1 if converged else 0
        if stable>=int(convergence["patience"]):stop="converged_configured_patience";break
    return current,{"algorithm":algorithm,"level_index":level_index,"parameters":level,"estimator":{"type":"explicit_region_GICP_covariance" if algorithm=="gicp" else "explicit_region_robust_point_to_plane","robust_kernel":None if algorithm=="gicp" else "HuberLoss","huber_k_m":level.get("huber_k_m")},
        "correspondence_policy":"board_to_board_and_object_to_object_only","joint_update":"one shared SE(3)","before_region_metrics":before,
        "after_region_metrics":diagnostic(current),"actual_iterations":len(traj),"maximum_iterations":int(level["max_iterations"]),"stop_reason":stop,"trajectory":traj}

def final_sanity(metrics:dict[str,Any],parent_metrics:dict[str,Any],gates:dict[str,Any])->list[str]:
    if not metrics.get("valid"):return ["invalid_final_transform"]
    b=metrics["board"];pb=parent_metrics["board"];reasons=[]
    if b["normal_angle_deg"]>float(gates["maximum_board_normal_angle_deg"]):reasons.append("final_board_normal_angle_exceeded")
    if b["plane_distance_m"]>float(gates["maximum_board_plane_distance_m"]):reasons.append("final_board_plane_distance_exceeded")
    if b["normal_angle_deg"]>pb["normal_angle_deg"]+float(gates["maximum_board_normal_worsening_deg"]):reasons.append("final_board_normal_vs_parent_worsened")
    if b["plane_distance_m"]>pb["plane_distance_m"]+float(gates["maximum_board_plane_distance_worsening_m"]):reasons.append("final_board_plane_vs_parent_worsened")
    return reasons
