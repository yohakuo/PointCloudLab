from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import open3d as o3d

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/"src"))
from pointcloud_registration.candidates import (candidate_id, deduplicate_candidates,
    generate_geometry_candidates, stage3_contract)
from pointcloud_registration.shape_grid import build_shape_grid, shape_score
from pointcloud_registration.transforms import (apply_transform, compose_plane_transform, right_handed_plane_frame,
    rotation_between_normals, transform_difference, validate_rigid_transform)

SPEC=importlib.util.spec_from_file_location("stage3",ROOT/"scripts"/"03_generate_candidates.py"); assert SPEC and SPEC.loader
SCRIPT=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(SCRIPT)


def cloud(p): return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(p,float)))
def square(z=0.,n=12):
    x,y=np.meshgrid(np.linspace(-.03,.03,n),np.linspace(-.03,.03,n));return np.c_[x.ravel(),y.ravel(),np.full(x.size,z)]


class StageThreeTests(unittest.TestCase):
    def test_shape_grid_holes_are_unknown_and_density_does_not_change_shape(self):
        cfg={"grid_size_m":.003,"confidence_saturation_points":3,"minimum_contour_support":.11,
             "sparse_cell_count_threshold":32,"outline_height_quantile":75,"outline_core_dilation_cells":1,
             "crop_margin_m":.004,"crop_confidence_factor":.15,"match_trim_fraction":.85,"minimum_reference_confidence":.5,
             "occupancy_weight":.35,"contour_weight":.55,"height_weight":.1,"height_clip_m":.02}
        ij=np.array([(i,j) for i in range(15) for j in range(15) if not (6<=i<=8 and 6<=j<=8)])
        xyz=np.c_[ij*.003+.0015,np.full(len(ij),.01)]
        frame=lambda p:{"uv":p[:,:2],"n":np.array([0.,0.,1.]),"plane":np.array([0.,0.,1.,0.])}
        base=np.repeat(xyz,4,axis=0)
        denser=np.vstack([base,np.repeat(xyz[:1],100,axis=0)])
        a=build_shape_grid(frame(base),base,cfg)
        b=build_shape_grid(frame(denser),denser,cfg)
        self.assertEqual(a["summary"]["observed_cells"],216)
        self.assertGreater(a["summary"]["unknown_cells_in_grid"],9)
        self.assertTrue(np.array_equal(a["centers"],b["centers"]))
        self.assertTrue(np.array_equal(a["contour"],b["contour"]))
        self.assertAlmostEqual(shape_score(a,b,0,np.zeros(2),cfg)[0],0.,places=12)
        self.assertTrue(np.all((a["contour"][:,0]<.018)|(a["contour"][:,0]>.03)|
                               (a["contour"][:,1]<.018)|(a["contour"][:,1]>.03)))
        cropped=build_shape_grid(frame(base),base,cfg,([0,0,0],[.045,.045,.02]))
        self.assertGreater(cropped["summary"]["crop_downweighted_cells"],0)
        self.assertLess(cropped["confidence"][cropped["crop_cell"]].mean(),
                        a["confidence"][cropped["crop_cell"]].mean())

    def test_direction_source_to_target(self):
        t=np.eye(4);t[:3,3]=[1,2,3]
        np.testing.assert_allclose(apply_transform([[.1,.2,.3]],t),[[1.1,2.2,3.3]])

    def test_metric_input_not_rescaled(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/"x.ply"; pts=np.array([[1.5,-.02,.1],[1.6,.01,.2]])
            o3d.io.write_point_cloud(str(p),cloud(pts)); loaded,_=SCRIPT.load_metric_stage2_cloud(p)
            np.testing.assert_allclose(np.asarray(loaded.points),pts)

    def test_normal_rotation_parallel_antiparallel_and_frame(self):
        for target in ([0,0,1],[0,0,-1],[1,0,0]):
            r,_=rotation_between_normals(np.array([0.,0.,1.]),np.array(target,float));np.testing.assert_allclose(r@np.array([0,0,1.]),target,atol=1e-7);self.assertAlmostEqual(np.linalg.det(r),1)
        f=right_handed_plane_frame(np.array([0,0,1,0.]),square(.01));self.assertAlmostEqual(f["determinant"],1.)
        f["plane"]=np.array([0,0,1,0.]);g=right_handed_plane_frame(np.array([0,0,1,-.5]),square(.51)+[1,2,0]);g["plane"]=np.array([0,0,1,-.5])
        t=compose_plane_transform(f,g,0,np.array([0.,0.]));self.assertAlmostEqual((t@np.r_[f["origin"],1.])[2],.5)

    def test_square_keeps_four_directions_multiple_translations_stable_ids(self):
        sf=right_handed_plane_frame(np.array([0,0,1,0.]),square(.01));sf["plane"]=np.array([0,0,1,0.])
        tf=right_handed_plane_frame(np.array([0,0,1,-.5]),square(.51)+[1,2,0]);tf["plane"]=np.array([0,0,1,-.5]);tf["normal_branch"]="test"
        cfg={"contour_sample_count":80,"contour_angle_step_deg":5.,"search_window_m":.003,"search_step_m":.003,"search_trim_fraction":.8,"search_peaks_per_direction":2,"search_nms_distance_m":.002,"minimum_valid_points":10}
        gate={"maximum_board_normal_angle_deg":40,"maximum_board_plane_distance_m":.03,"maximum_object_trimmed_chamfer_m":.08,"minimum_object_overlap_ratio_at_15mm":0.,"aabb_disconnection_margin_m":.1,"diagnostic_sample_limit":100,"trim_fraction":.8}
        a,_=generate_geometry_candidates(sf,tf,square(.01),square(.51)+[1,2,0],cfg,gate,{"a":"b"},"x");b,_=generate_geometry_candidates(sf,tf,square(.01),square(.51)+[1,2,0],cfg,gate,{"a":"b"},"x")
        self.assertEqual({x["provenance"]["discrete_increment_deg"] for x in a},{0,90,180,270});self.assertGreaterEqual(len(a),4*3*2);self.assertEqual([x["candidate_id"] for x in a],[x["candidate_id"] for x in b])

    def test_only_geometry_candidate_ids_are_supported(self):
        self.assertTrue(candidate_id("geometry", {"branch": 0}).startswith("s3_geo_"))
        with self.assertRaises(ValueError):
            candidate_id("fpfh_ransac", {"branch": 0})

    def test_invalid_transforms_rejected(self):
        cases=[]
        x=np.eye(4);x[0,0]=-1;cases.append(x)
        x=np.eye(4);x[0,0]=2;cases.append(x)
        x=np.eye(4);x[3,0]=1;cases.append(x)
        x=np.eye(4);x[0,0]=np.nan;cases.append(x)
        for x in cases:self.assertFalse(validate_rigid_transform(x)["valid"])

    def test_se3_dedup_preserves_members_and_90_180(self):
        def item(name,deg,t,gen="geometry",seed=None):
            a=np.deg2rad(deg);m=np.eye(4);m[:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]];m[:3,3]=t
            return {"candidate_id":name,"generator":gen,"matrix_m":m.tolist(),"status":"retained_raw","coarse_rank_score":0.,"provenance":{"seed":seed}}
        data=[item("a",0,[0,0,0]),item("b",1,[.001,0,0],"geometry",42),item("c",90,[0,0,0]),item("d",180,[0,0,0])]
        can,audit=deduplicate_candidates(data,3,.006,10);self.assertEqual(len(can),3);self.assertEqual(can[0]["support_count"],2);self.assertFalse(can[0]["cross_generator_support"]);self.assertIn("b",can[0]["member_candidate_ids"])
        self.assertAlmostEqual(transform_difference(np.asarray(data[0]["matrix_m"]),np.asarray(data[2]["matrix_m"]))[0],90.)

    def test_contract_forbids_later_stages(self):
        c=stage3_contract();self.assertEqual(c["transform_direction"],"INSPIRE_TO_FAST");self.assertFalse(c["refinement_performed"]);self.assertFalse(c["final_transform_selected"]);self.assertFalse(c["full_cloud_transformed"])

    def test_json_csv_matrix_serialization_consistency(self):
        with tempfile.TemporaryDirectory() as d:
            out=Path(d);(out/"candidate_matrices").mkdir();m=np.eye(4)
            candidate={"candidate_id":"s3_can_test","rank":1,"generator":"geometry","generators":["geometry"],"support_count":1,
                "status":"canonical","coarse_rank_score":0.,"matrix_m":m.tolist(),"coarse_checks":{"board":{"normal_angle_deg":0.,"plane_distance_m":0.},"object":{"bidirectional_trimmed_chamfer_m":0.,"bidirectional_overlap_at_15mm":1.}}}
            (out/"canonical_candidates.json").write_text(json.dumps({"candidate_count":1,"candidates":[candidate]}),encoding="utf-8")
            SCRIPT.write_csv(out/"candidate_summary.csv",[candidate]);np.savetxt(out/"candidate_matrices"/"s3_can_test.txt",m,fmt="%.17g")
            self.assertTrue(SCRIPT.validate_serialized_outputs(out,[candidate])["passed"])


if __name__=="__main__":unittest.main()

