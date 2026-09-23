import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import open3d as o3d

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))

from pointcloud_registration.export import (CORE_FILES,VISUAL_FILES,ExportError,atomic_write_cloud,
    cloud_stats,load_selected_refinement,merge_clouds,raw_mm_affine,stage5_contract,
    transform_source_cloud,verify_cloud_roundtrip,verify_transform_paths)
from pointcloud_registration.io_utils import sha256_file
from pointcloud_registration.transforms import apply_transform,validate_rigid_transform


def cloud(points, colors=None, normals=None):
    c=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(points,float)))
    if colors is not None:c.colors=o3d.utility.Vector3dVector(np.asarray(colors,float))
    if normals is not None:c.normals=o3d.utility.Vector3dVector(np.asarray(normals,float))
    return c


class StageFiveTests(unittest.TestCase):
    def test_parent_unique_selects_matrix_m_not_parent(self):
        with tempfile.TemporaryDirectory() as td:
            d=Path(td);(d/"m").mkdir();m=np.eye(4);m[0,3]=2.;parent=np.eye(4);parent[0,3]=9.
            diagnostic=np.eye(4);diagnostic[1,3]=77.
            payload={"stage":4,"status":"success","transform_direction":"INSPIRE_TO_FAST","unit":"m","candidates":[
                {"parent_candidate_id":"p","refined_candidate_id":"r","status":"refined_valid","matrix_m":m.tolist(),"parent_matrix_m":parent.tolist(),
                 "visualization_matrix_m":diagnostic.tolist(),"visualization_matrix_role":"diagnostic_only"}]}
            (d/"r.json").write_text(json.dumps(payload));np.savetxt(d/"m"/"r.txt",m,fmt="%.17g")
            with (d/"s.csv").open("w",newline="") as f:
                w=csv.DictWriter(f,fieldnames=["parent_candidate_id","refined_candidate_id","matrix_file"]);w.writeheader();w.writerow({"parent_candidate_id":"p","refined_candidate_id":"r","matrix_file":"refined_matrices/r.txt"})
            chosen,got,audit=load_selected_refinement(d/"r.json",d/"s.csv",d/"m","p","r")
            self.assertTrue(np.array_equal(got,m));self.assertFalse(np.array_equal(got,parent));self.assertIn("parent_matrix_m explicitly forbidden",audit["matrix_source"])
            self.assertFalse(np.array_equal(got,diagnostic));self.assertEqual(audit["matrix_source"],"matrix_m (parent_matrix_m explicitly forbidden)")

    def test_non_unique_or_invalid_status_fails(self):
        with tempfile.TemporaryDirectory() as td:
            d=Path(td);(d/"m").mkdir();m=np.eye(4);row={"parent_candidate_id":"p","refined_candidate_id":"r","status":"refined_valid","matrix_m":m.tolist()}
            (d/"r.json").write_text(json.dumps({"stage":4,"status":"success","transform_direction":"INSPIRE_TO_FAST","unit":"m","candidates":[row,row]}))
            (d/"s.csv").write_text("parent_candidate_id,refined_candidate_id,matrix_file\n")
            with self.assertRaises(ExportError):load_selected_refinement(d/"r.json",d/"s.csv",d/"m","p")

    def test_explicit_override_allows_nonvalid_matrix_m_and_records_audit(self):
        with tempfile.TemporaryDirectory() as td:
            d=Path(td);(d/"m").mkdir();m=np.eye(4);m[0,3]=2.
            row={"parent_candidate_id":"p","refined_candidate_id":"r","status":"no_safe_refinement","matrix_m":m.tolist()}
            (d/"r.json").write_text(json.dumps({"stage":4,"status":"success","transform_direction":"INSPIRE_TO_FAST","unit":"m","candidates":[row]}))
            np.savetxt(d/"m"/"r.txt",m,fmt="%.17g")
            (d/"s.csv").write_text("parent_candidate_id,refined_candidate_id,matrix_file\np,r,refined_matrices/r.txt\n")
            with self.assertRaises(ExportError):
                load_selected_refinement(d/"r.json",d/"s.csv",d/"m","p","r")
            chosen,got,audit=load_selected_refinement(d/"r.json",d/"s.csv",d/"m","p","r",allow_nonvalid_selection=True)
            self.assertEqual(chosen["status"],"no_safe_refinement");self.assertTrue(np.array_equal(got,m))
            self.assertTrue(audit["safety_gate_overridden"]);self.assertTrue(audit["allow_nonvalid_selection"])

    def test_direction_and_target_unchanged(self):
        t=np.eye(4);t[:3,3]=[1,2,3];src=np.array([[1000,0,0.]]);target=np.array([[7.,8.,9.]])
        reg,_=transform_source_cloud(cloud(src),t)
        self.assertTrue(np.allclose(np.asarray(reg.points),[[2,2,3]]));self.assertTrue(np.array_equal(target,[[7,8,9]]))

    def test_mm_to_m_once_about_origin(self):
        src=cloud([[1000,-2000,3000],[2000,-4000,6000]])
        reg,meta=transform_source_cloud(src,np.eye(4))
        self.assertTrue(np.array_equal(np.asarray(reg.points),[[1,-2,3],[2,-4,6]]));self.assertEqual(meta["unit_conversion_count"],1)

    def test_raw_affine_relation_and_two_paths(self):
        t=np.eye(4);t[:3,3]=[.2,-.1,.3];raw=raw_mm_affine(t)
        self.assertTrue(np.array_equal(raw,t@np.diag([.001,.001,.001,1.])))
        result=verify_transform_paths(np.array([[0,0,0],[1000,2,-5.]]),t,raw,1e-14,10,42)
        self.assertTrue(result["passed"]);self.assertAlmostEqual(np.linalg.det(raw[:3,:3]),1e-9)

    def test_rigid_rejects_bad_last_row_reflection_scale_and_nonfinite(self):
        variants=[]
        x=np.eye(4);x[3,0]=1;variants.append(x)
        x=np.eye(4);x[0,0]=-1;variants.append(x)
        x=np.eye(4);x[0,0]=2;variants.append(x)
        x=np.eye(4);x[0,0]=np.nan;variants.append(x)
        x=np.eye(4);x[0,0]=np.inf;variants.append(x)
        self.assertTrue(validate_rigid_transform(np.eye(4),1e-10)["valid"])
        self.assertTrue(all(not validate_rigid_transform(x,1e-10)["valid"] for x in variants))

    def test_points_normals_colors_policy(self):
        theta=np.pi/2;r=np.array([[0,-1,0],[1,0,0],[0,0,1.] ]);t=np.eye(4);t[:3,:3]=r;t[:3,3]=[1,0,0]
        c=cloud([[1000,0,0]],[[.2,.3,.4]],[[1,0,0]])
        out,_=transform_source_cloud(c,t)
        self.assertTrue(np.allclose(np.asarray(out.points),[[1,1,0]]));self.assertTrue(np.allclose(np.asarray(out.normals),[[0,1,0]]));self.assertTrue(np.array_equal(np.asarray(out.colors),[[.2,.3,.4]]))

    def test_merge_never_transforms_target_and_compatibility(self):
        target=cloud([[9,8,7]],[[1,0,0]]);reg=cloud([[1,2,3]],[[0,1,0]],[[0,0,1]])
        merged,policy=merge_clouds(target,reg)
        self.assertTrue(np.array_equal(np.asarray(merged.points),[[9,8,7],[1,2,3]]));self.assertTrue(merged.has_colors());self.assertFalse(merged.has_normals());self.assertFalse(policy["target_coordinates_transformed"])

    def test_ply_roundtrip_count_coordinates_attributes(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/"x.ply";c=cloud([[1,2,3],[4,5,6]],[[1,0,0],[0,1,0]],[[0,0,1],[0,1,0]])
            atomic_write_cloud(p,c);v=verify_cloud_roundtrip(p,c,1e-12,10,1)
            self.assertEqual(v["reread_stats"]["point_count"],2);self.assertTrue(v["reread_stats"]["has_colors"]);self.assertTrue(v["reread_stats"]["has_normals"])

    def test_input_hash_unchanged(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/"input.bin";p.write_bytes(b"fixed");before=sha256_file(p)
            transform_source_cloud(cloud([[1,2,3]]),np.eye(4));self.assertEqual(before,sha256_file(p))

    def test_export_contract_has_no_invented_ranking_results(self):
        c=stage5_contract("p","r")
        self.assertFalse(c["automatic_scoring_performed"]);self.assertFalse(c["automatic_ranking_performed"]);self.assertFalse(c["ambiguity_adjudication_performed"]);self.assertFalse(c["transform_uniqueness_determined"])

    def test_required_deliverable_names(self):
        self.assertEqual(len(CORE_FILES),5);self.assertEqual(len(VISUAL_FILES),7)
        self.assertIn("registration_report.json",CORE_FILES);self.assertIn("distance_colormap.png",VISUAL_FILES)

    def test_matrix_report_values_json_serializable(self):
        t=raw_mm_affine(np.eye(4));roundtrip=np.asarray(json.loads(json.dumps({"m":t.tolist()}))["m"])
        self.assertTrue(np.array_equal(t,roundtrip))

    def test_cloud_stats_fails_nonfinite(self):
        with self.assertRaises(ExportError):cloud_stats(cloud([[np.nan,0,0]]))

    def test_cli_failure_returns_nonzero_and_failed_report(self):
        spec=importlib.util.spec_from_file_location("stage5_cli",ROOT/"scripts"/"05_export_results.py")
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as td:
            out=Path(td)/"out"
            code=module.main(["--stage1-report",str(Path(td)/"missing.json"),"--output-dir",str(out)])
            self.assertNotEqual(code,0)
            report=json.loads((out/"registration_report.json").read_text(encoding="utf8"))
            self.assertEqual(report["status"],"failed");self.assertTrue(report["failure_reasons"])


if __name__=="__main__":unittest.main()

