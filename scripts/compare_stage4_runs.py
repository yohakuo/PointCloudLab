#!/usr/bin/env python3
"""Create an auditable before/after comparison for two formal stage-5 reports."""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from statistics import mean, median
from typing import Any


ROOT=Path(__file__).resolve().parents[1]


def parser()->argparse.ArgumentParser:
    p=argparse.ArgumentParser()
    p.add_argument("--before",type=Path,default=ROOT/"outputs"/"stage_04_refinement_before_step_scaling"/"refinement_report.json")
    p.add_argument("--after",type=Path,default=ROOT/"outputs"/"stage_04_refinement"/"refinement_report.json")
    p.add_argument("--output",type=Path,default=ROOT/"outputs"/"stage_04_refinement"/"stage4_before_after_comparison.json")
    p.add_argument("--markdown",type=Path,default=ROOT/"outputs"/"stage_04_refinement"/"stage4_before_after_comparison.md")
    return p


def load(path:Path)->dict[str,Any]:return json.loads(path.read_text(encoding="utf-8"))


def rows(report:dict[str,Any]):
    return [row for c in report["refined_candidates"] for level in c["levels"] for row in level["trajectory"]]


def frequencies(report:dict[str,Any])->dict[str,int]:
    return dict(sorted(Counter(reason for row in rows(report) if not row.get("accepted",False) for reason in row.get("rejection_reasons",[])).items()))


def stats(values:list[float],scale:float=1.)->dict[str,float|int|None]:
    x=[float(v)*scale for v in values]
    return {"count":len(x),"minimum":min(x) if x else None,"median":median(x) if x else None,
        "mean":mean(x) if x else None,"maximum":max(x) if x else None}


def formal_inplane_shift(candidate:dict[str,Any])->float:
    if candidate["status"]!="refined_valid":return 0.
    accepted=[row for level in candidate["levels"] for row in level["trajectory"] if row.get("accepted")]
    return float(accepted[-1].get("object_centroid_inplane_shift_from_parent_m") or 0.) if accepted else 0.


def metric_summary(report:dict[str,Any],valid_only:bool)->dict[str,Any]:
    cs=[c for c in report["refined_candidates"] if not valid_only or c["status"]=="refined_valid"]
    return {"candidate_count":len(cs),
        "object_trimmed_distance_mm":stats([c["final_metrics"]["object"]["bidirectional_trimmed_distance_m"] for c in cs],1000),
        "object_overlap_2_5mm":stats([c["final_metrics"]["object"]["bidirectional_overlap_ratios"]["at_2.5mm"] for c in cs]),
        "object_overlap_5mm":stats([c["final_metrics"]["object"]["bidirectional_overlap_ratios"]["at_5mm"] for c in cs]),
        "board_normal_angle_deg":stats([c["final_metrics"]["board"]["normal_angle_deg"] for c in cs]),
        "board_plane_distance_mm":stats([c["final_metrics"]["board"]["plane_distance_m"] for c in cs],1000),
        "formal_object_centroid_inplane_shift_from_parent_mm":stats([formal_inplane_shift(c) for c in cs],1000)}


def snapshot(report:dict[str,Any])->dict[str,Any]:
    threshold=float(report["config"]["sanity_gates"]["maximum_object_centroid_inplane_shift_m"])
    per_candidate=[]
    for c in report["refined_candidates"]:
        m=c["final_metrics"];shift=formal_inplane_shift(c)
        per_candidate.append({"parent_candidate_id":c["parent_candidate_id"],"status":c["status"],
            "object_trimmed_distance_mm":1000*m["object"]["bidirectional_trimmed_distance_m"],
            "object_overlap_2_5mm":m["object"]["bidirectional_overlap_ratios"]["at_2.5mm"],
            "object_overlap_5mm":m["object"]["bidirectional_overlap_ratios"]["at_5mm"],
            "board_normal_angle_deg":m["board"]["normal_angle_deg"],"board_plane_distance_mm":1000*m["board"]["plane_distance_m"],
            "parent_rotation_drift_deg":c["from_parent_rotation_deg"],"parent_translation_drift_mm":1000*c["from_parent_translation_m"],
            "object_centroid_inplane_shift_from_parent_mm":1000*shift})
    return {"candidate_counts":report["candidate_counts"],"rejection_reason_frequencies":frequencies(report),
        "metrics_all_formal_candidates":metric_summary(report,False),"metrics_refined_valid_only":metric_summary(report,True),
        "maximum_allowed_inplane_shift_mm":1000*threshold,
        "obvious_inplane_sliding_detected":any(x["object_centroid_inplane_shift_from_parent_mm"]>1000*threshold+1e-12 for x in per_candidate),
        "per_candidate":per_candidate,"backtracking_summary":report.get("backtracking_summary")}


def fmt(x:float|None,digits:int=4)->str:return "n/a" if x is None else f"{x:.{digits}f}"


def main()->int:
    args=parser().parse_args();before=load(args.before.resolve());after=load(args.after.resolve())
    b=snapshot(before);a=snapshot(after);bm=b["metrics_refined_valid_only"];am=a["metrics_refined_valid_only"]
    before_by={x["parent_candidate_id"]:x for x in b["per_candidate"]};after_by={x["parent_candidate_id"]:x for x in a["per_candidate"]}
    changed=[{"parent_candidate_id":key,"before_status":before_by[key]["status"],"after_status":after_by[key]["status"]}
        for key in before_by if before_by[key]["status"]!=after_by[key]["status"]]
    bg=before["config"]["sanity_gates"];ag=after["config"]["sanity_gates"]
    policy_keys={"maximum_step_translation_m","maximum_step_translation_by_voxel_m","translation_backtracking_factor","translation_backtracking_max_attempts"}
    unchanged={key:{"before":bg[key],"after":ag[key]} for key in bg.keys()&ag.keys() if key not in policy_keys and bg[key]==ag[key]}
    changed_nonpolicy={key:{"before":bg.get(key),"after":ag.get(key)} for key in (bg.keys()|ag.keys())-policy_keys if bg.get(key)!=ag.get(key)}
    payload={"schema":"pointcloudlab.stage4.before_after_comparison","version":1,
        "before_report":str(args.before.resolve()),"after_report":str(args.after.resolve()),"before":b,"after":a,"status_changes":changed,
        "safety_conclusion":{"unchanged_non_translation_policy_gates":unchanged,"changed_non_translation_policy_gates":changed_nonpolicy,
            "cumulative_drift_limits_unchanged":bg["maximum_parent_rotation_drift_deg"]==ag["maximum_parent_rotation_drift_deg"]==3.
                and bg["maximum_parent_translation_drift_m"]==ag["maximum_parent_translation_drift_m"]==.006,
            "regional_geometry_gates_unchanged":not changed_nonpolicy,
            "no_formal_candidate_exceeds_parent_rotation_limit":all(x["parent_rotation_drift_deg"]<=3.+1e-12 for x in a["per_candidate"]),
            "no_formal_candidate_exceeds_parent_translation_limit":all(x["parent_translation_drift_mm"]<=6.+1e-12 for x in a["per_candidate"]),
            "no_obvious_inplane_sliding":not a["obvious_inplane_sliding_detected"]}}
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding="utf-8")
    lines=["# 阶段 4 单步平移门槛修改前后对比","",
        f"- refined_valid：{b['candidate_counts']['refined_valid']} → {a['candidate_counts']['refined_valid']}",
        f"- no_safe_refinement：{b['candidate_counts']['no_safe_refinement']} → {a['candidate_counts']['no_safe_refinement']}",
        f"- refinement_failed：{b['candidate_counts']['refinement_failed']} → {a['candidate_counts']['refinement_failed']}",
        f"- 正式有效候选对象 trimmed 距离均值：{fmt(bm['object_trimmed_distance_mm']['mean'])} → {fmt(am['object_trimmed_distance_mm']['mean'])} mm",
        f"- 正式有效候选 5 mm 重叠率均值：{fmt(bm['object_overlap_5mm']['mean'])} → {fmt(am['object_overlap_5mm']['mean'])}",
        f"- 正式有效候选白板法向角均值：{fmt(bm['board_normal_angle_deg']['mean'])} → {fmt(am['board_normal_angle_deg']['mean'])}°",
        f"- 正式有效候选白板面距均值：{fmt(bm['board_plane_distance_mm']['mean'])} → {fmt(am['board_plane_distance_mm']['mean'])} mm",
        f"- 修改后正式候选最大平面内质心位移：{fmt(a['metrics_all_formal_candidates']['formal_object_centroid_inplane_shift_from_parent_mm']['maximum'])} mm（门槛 {a['maximum_allowed_inplane_shift_mm']:.1f} mm）",
        f"- 明显平面内滑动：{'是' if a['obvious_inplane_sliding_detected'] else '否'}","",
        "完整逐候选指标、拒绝原因频次及回溯统计见同名 JSON。"]
    args.markdown.write_text("\n".join(lines)+"\n",encoding="utf-8");return 0


if __name__=="__main__":raise SystemExit(main())

