from __future__ import annotations
import importlib.util,json,sys,tempfile,unittest
from pathlib import Path
import numpy as np
import open3d as o3d
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/"src"))
from pointcloud_registration.refinement import (prepare_balanced_level,refined_candidate_id,region_diagnostics,run_level,
    stage4_contract,step_translation_limit,transform_increment,validate_level_configuration,_explicit_region_iteration)
from pointcloud_registration.transforms import apply_transform,validate_rigid_transform
from pointcloud_registration.visualization import _stage4_candidate_title,_stage4_status_presentation
SPEC=importlib.util.spec_from_file_location("stage4",ROOT/"scripts"/"04_refine_candidates.py");SCRIPT=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(SCRIPT)

def cloud(x):return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(x,float)))
def plane(n=12,z=0):
    x,y=np.meshgrid(np.linspace(-.03,.03,n),np.linspace(-.03,.03,n));return np.c_[x.ravel(),y.ravel(),np.full(x.size,z)]

class StageFourTests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads((ROOT/"configs"/"registration.yaml").read_text(encoding="utf-8"))["refinement"]
        self.gates=self.cfg["sanity_gates"];self.conv=self.cfg["convergence"]
        self.regions,_=prepare_balanced_level(cloud(plane()),cloud(plane(9,.005)),.003,self.cfg["sampling"],self.cfg["normal_estimation"],self.cfg["covariance_estimation"],42)
    def metrics(self,trim=.004,overlap=.5,board_distance=0.,board_angle=0.):
        return {"valid":True,"board":{"normal_angle_deg":board_angle,"plane_distance_m":board_distance},"object":{
            "bidirectional_trimmed_distance_m":trim,"bidirectional_distance_percentiles":{"p50_m":trim,"p80_m":trim,"p90_m":trim},
            "bidirectional_overlap_ratios":{"at_2.5mm":overlap,"at_5mm":overlap},"trim_fraction":.9}}
    def result(self,t,board=100,obj=80):
        return {"transformation":t,"regions":{"board":{"source_point_count":120,"target_point_count":120,"correspondence_count":board,"correspondence_ratio":board/120,"rmse_m":.001,"robust_weight":{}},
            "object":{"source_point_count":90,"target_point_count":90,"correspondence_count":obj,"correspondence_ratio":obj/90,"rmse_m":.001,"robust_weight":{}}},
            "joint_cost":1.,"normal_equation_condition":10.}

    def test_contract_direction_lineage_and_no_selection(self):
        parents=[f"s3_can_{i:012x}" for i in range(16)];ids=[refined_candidate_id(x) for x in parents]
        self.assertEqual(len(set(ids)),16);self.assertEqual(stage4_contract()["transform_direction"],"INSPIRE_TO_FAST")
        self.assertFalse(stage4_contract()["final_transform_selected"])

    def test_configuration_has_requested_tight_gates(self):
        validate_level_configuration(self.cfg);g=self.gates
        self.assertEqual(g["maximum_step_rotation_deg"],.5)
        self.assertNotIn("maximum_step_translation_m",g)
        expected={.005:.0025,.004:.002,.003:.0015}
        for level in self.cfg["gicp_levels"]+self.cfg["robust_point_to_plane_levels"]:
            self.assertEqual(step_translation_limit(level,g),expected[level["voxel_size_m"]])
        self.assertEqual(g["maximum_parent_rotation_drift_deg"],3.);self.assertEqual(g["maximum_parent_translation_drift_m"],.006)
        self.assertEqual(g["object_trim_fraction"],.9);self.assertEqual(g["object_overlap_thresholds_m"],[.0025,.005])

    def test_labeled_preparation_never_combines_regions(self):
        self.assertEqual(set(self.regions),{"board","object"});_,report=prepare_balanced_level(cloud(plane(20)),cloud(plane(9,.005)),.004,
            self.cfg["sampling"],self.cfg["normal_estimation"],self.cfg["covariance_estimation"],9)
        self.assertTrue(report["explicit_region_labels"]);self.assertFalse(report["cross_region_correspondence_possible"])

    def test_explicit_solver_reports_each_region_and_one_joint_update(self):
        out=_explicit_region_iteration(self.regions,self.regions,.01,np.eye(4),"gicp",self.cfg["gicp_levels"][0])
        self.assertEqual(set(out["regions"]),{"board","object"});self.assertIn("equal-region",out["solver"])
        self.assertTrue(validate_rigid_transform(out["transformation"])["valid"])

    def test_region_diagnostics_records_percentiles_and_two_overlaps(self):
        d=region_diagnostics(np.eye(4),plane(),plane(8,.01),plane(),plane(8,.01),.9,[.0025,.005])
        self.assertEqual(set(d["object"]["bidirectional_distance_percentiles"]),{"p50_m","p80_m","p90_m"})
        self.assertEqual(set(d["object"]["bidirectional_overlap_ratios"]),{"at_2.5mm","at_5mm"})

    def test_oversized_translation_is_scaled_and_rechecked_before_acceptance(self):
        proposed=np.eye(4);proposed[0,3]=.003
        diag=lambda t:self.metrics(trim=.003 if t[0,3] else .004,overlap=.6 if t[0,3] else .5)
        final,rec=run_level(self.regions,self.regions,np.eye(4),np.eye(4),"gicp",1,{**self.cfg["gicp_levels"][0],"max_iterations":1},self.conv,self.gates,diag,
            plane(8,.005),plane(),lambda *x:self.result(proposed))
        row=rec["trajectory"][0]
        self.assertTrue(row["accepted"]);self.assertTrue(row["backtracking_performed"])
        self.assertAlmostEqual(row["raw_delta_translation_m"],.003);self.assertAlmostEqual(row["delta_translation_m"],.0025)
        self.assertAlmostEqual(row["step_scale_ratio"],5/6);self.assertEqual(row["final_decision_reason"],"accepted_after_translation_backtracking")
        self.assertIn("step_translation_trust_region_exceeded",row["raw_proposal_rejection_reasons"])
        np.testing.assert_allclose(final[0,3],.0025)

    def test_backtracking_rechecks_geometry_and_rejects_when_all_scales_fail(self):
        proposed=np.eye(4);proposed[2,3]=.003
        diag=lambda t:self.metrics(trim=.003 if t[2,3] else .004,overlap=.6 if t[2,3] else .5,
            board_distance=.0031 if t[2,3] else .001)
        final,rec=run_level(self.regions,self.regions,np.eye(4),np.eye(4),"gicp",1,self.cfg["gicp_levels"][0],self.conv,self.gates,diag,
            plane(8,.005),plane(),lambda *x:self.result(proposed))
        row=rec["trajectory"][0]
        np.testing.assert_allclose(final,np.eye(4));self.assertEqual(len(row["backtracking_attempts"]),6)
        self.assertEqual(row["final_decision_reason"],"rejected_after_translation_backtracking_exhausted")
        self.assertIn("translation_backtracking_exhausted",row["rejection_reasons"])
        self.assertTrue(all("board_plane_absolute_limit_exceeded" in x["rejection_reasons"] for x in row["backtracking_attempts"]))

    def test_backtracking_does_not_bypass_parent_cumulative_drift(self):
        current=np.eye(4);current[2,3]=.0058;proposed=np.eye(4);proposed[2,3]=.0088
        diag=lambda t:self.metrics(trim=.0038 if t[2,3]>.005800001 else .004,overlap=.51 if t[2,3]>.005800001 else .5)
        final,rec=run_level(self.regions,self.regions,current,np.eye(4),"gicp",1,{**self.cfg["gicp_levels"][0],"max_iterations":1},self.conv,self.gates,diag,
            plane(8,.005),plane(),lambda *x:self.result(proposed))
        attempts=rec["trajectory"][0]["backtracking_attempts"]
        self.assertTrue(any("parent_translation_trust_region_exceeded" in x["rejection_reasons"] for x in attempts))
        self.assertLessEqual(np.linalg.norm(final[:3,3]),.006)
        self.assertTrue(attempts[-1]["accepted"])

    def test_backtracking_does_not_bypass_inplane_centroid_gate(self):
        angle=np.deg2rad(.4);proposed=np.eye(4);proposed[:2,:2]=[[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]];proposed[2,3]=.003
        diag=lambda t:self.metrics(trim=.0038 if not np.array_equal(t,np.eye(4)) else .004,
            overlap=.51 if not np.array_equal(t,np.eye(4)) else .5)
        far_object=plane(8,.005)+np.array([1.,0.,0.])
        final,rec=run_level(self.regions,self.regions,np.eye(4),np.eye(4),"gicp",1,{**self.cfg["gicp_levels"][0],"max_iterations":1},self.conv,self.gates,diag,
            far_object,plane(),lambda *x:self.result(proposed))
        attempts=rec["trajectory"][0]["backtracking_attempts"]
        self.assertIn("object_centroid_inplane_shift_exceeded",attempts[0]["rejection_reasons"])
        self.assertTrue(attempts[-1]["accepted"])
        self.assertLessEqual(attempts[-1]["object_centroid_inplane_shift_from_parent_m"],.005)
        self.assertFalse(np.array_equal(final,np.eye(4)))

    def test_rotation_limit_is_unchanged(self):
        angle=np.deg2rad(.6);proposed=np.eye(4);proposed[:2,:2]=[[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]
        diag=lambda t:self.metrics(trim=.003 if not np.array_equal(t,np.eye(4)) else .004,overlap=.6 if not np.array_equal(t,np.eye(4)) else .5)
        final,rec=run_level(self.regions,self.regions,np.eye(4),np.eye(4),"gicp",1,self.cfg["gicp_levels"][0],self.conv,self.gates,diag,
            plane(8,.005),plane(),lambda *x:self.result(proposed))
        np.testing.assert_allclose(final,np.eye(4));self.assertIn("step_rotation_trust_region_exceeded",rec["trajectory"][0]["rejection_reasons"])

    def test_parent_board_and_absolute_limits_are_checked_every_update(self):
        proposed=np.eye(4);proposed[2,3]=.0005
        diag=lambda t:self.metrics(trim=.003 if t[2,3] else .004,overlap=.6 if t[2,3] else .5,board_distance=.0031 if t[2,3] else .001)
        _,rec=run_level(self.regions,self.regions,np.eye(4),np.eye(4),"gicp",1,self.cfg["gicp_levels"][0],self.conv,self.gates,diag,
            plane(8,.005),plane(),lambda *x:self.result(proposed))
        reasons=rec["trajectory"][0]["rejection_reasons"];self.assertIn("board_plane_vs_parent_worsened",reasons);self.assertIn("board_plane_absolute_limit_exceeded",reasons)

    def test_both_region_correspondence_minima_are_enforced(self):
        diag=lambda t:self.metrics(trim=.003,overlap=.6)
        _,rec=run_level(self.regions,self.regions,np.eye(4),np.eye(4),"gicp",1,self.cfg["gicp_levels"][0],self.conv,self.gates,diag,
            plane(8,.005),plane(),lambda *x:self.result(np.eye(4),board=49,obj=19))
        reasons=rec["trajectory"][0]["rejection_reasons"];self.assertIn("board_correspondence_below_minimum",reasons);self.assertIn("object_correspondence_below_minimum",reasons)

    def test_object_requires_clear_improvement(self):
        diag=lambda t:self.metrics(trim=.004,overlap=.5)
        _,rec=run_level(self.regions,self.regions,np.eye(4),np.eye(4),"gicp",1,self.cfg["gicp_levels"][0],self.conv,self.gates,diag,
            plane(8,.005),plane(),lambda *x:self.result(np.eye(4)))
        self.assertIn("object_has_no_clear_improvement",rec["trajectory"][0]["rejection_reasons"])

    def test_object_parent_worsening_and_centroid_shift_are_rejected(self):
        proposed=np.eye(4);proposed[0,3]=.0008
        # Current/parent diagnostics are good, proposal improves overlap but violates tight trimmed non-degradation.
        diag=lambda t:self.metrics(trim=.0044 if t[0,3] else .004,overlap=.51 if t[0,3] else .5)
        _,rec=run_level(self.regions,self.regions,np.eye(4),np.eye(4),"gicp",1,self.cfg["gicp_levels"][0],self.conv,self.gates,diag,
            plane(8,.005),plane(),lambda *x:self.result(proposed))
        self.assertIn("object_trimmed_vs_parent_worsened",rec["trajectory"][0]["rejection_reasons"])

    def test_safe_improvement_is_accepted(self):
        proposed=np.eye(4);proposed[0,3]=.0002
        diag=lambda t:self.metrics(trim=.0038 if t[0,3] else .004,overlap=.51 if t[0,3] else .5)
        final,rec=run_level(self.regions,self.regions,np.eye(4),np.eye(4),"gicp",1,{**self.cfg["gicp_levels"][0],"max_iterations":1},self.conv,self.gates,diag,
            plane(8,.005),plane(),lambda *x:self.result(proposed))
        self.assertTrue(rec["trajectory"][0]["accepted"]);np.testing.assert_allclose(final,proposed)

    def test_transform_delta_and_illegal_rigid(self):
        old=np.eye(4);old[0,3]=1;new=np.eye(4);new[0,3]=1.25
        self.assertAlmostEqual(transform_increment(new,old)[2],.25)
        bad=np.eye(4);bad[0,0]=-1;self.assertFalse(validate_rigid_transform(bad)["valid"])

    def test_metric_input_and_protected_hash_are_unchanged(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/"source.ply";pts=np.array([[1.5,-.02,.1],[1.6,.01,.2],[1.7,0,.3]])
            o3d.io.write_point_cloud(str(p),cloud(pts));_,loaded=SCRIPT.load_metric(p);np.testing.assert_allclose(loaded,pts)
            self.assertEqual(SCRIPT.integrity({"x":p}),SCRIPT.integrity({"x":p}))

    def diagnostic_candidate(self,status="no_safe_refinement",proposals=None,matrix=None):
        parent=np.eye(4);formal=parent.copy() if matrix is None else matrix
        trajectory=[]
        for iteration,(proposed,accepted,reasons) in enumerate(proposals or [],1):
            trajectory.append({"iteration":iteration,"accepted":accepted,"rejection_reasons":reasons,
                "proposed_matrix_m":np.asarray(proposed).tolist(),"matrix_m":parent.tolist()})
        return {"refined_candidate_id":"r","parent_candidate_id":"p","status":status,
            "parent_matrix_m":parent.tolist(),"matrix_m":np.asarray(formal).tolist(),
            "failure_reasons":["formal_gate_failed"],"levels":[{"algorithm":"gicp","level_index":1,"trajectory":trajectory}]}

    def test_rejected_formal_matrix_stays_parent_but_visualization_uses_last_proposal(self):
        first=np.eye(4);first[0,3]=.0002;last=np.eye(4);last[1,3]=.0007
        c=self.diagnostic_candidate(proposals=[(first,False,["first_rejection"]),(last,False,["last_rejection"])])
        record=SCRIPT.select_visualization_record(c)
        self.assertTrue(np.array_equal(np.asarray(c["matrix_m"]),np.asarray(c["parent_matrix_m"])))
        np.testing.assert_array_equal(record["visualization_matrix_m"],last)
        self.assertEqual(record["visualization_source_iteration"],2);self.assertFalse(record["visualization_proposal_accepted"])
        self.assertEqual(record["visualization_rejection_reasons"],["last_rejection"])

    def test_nonfinite_and_nonrigid_proposals_are_never_visualized(self):
        legal=np.eye(4);legal[2,3]=.0003
        nan=np.eye(4);nan[0,0]=np.nan
        inf=np.eye(4);inf[0,3]=np.inf
        scale=np.eye(4);scale[0,0]=2.
        c=self.diagnostic_candidate(status="refinement_failed",proposals=[(legal,False,["legal"]),(nan,False,[]),(inf,False,[]),(scale,False,[])])
        record=SCRIPT.select_visualization_record(c)
        np.testing.assert_array_equal(record["visualization_matrix_m"],legal)
        self.assertTrue(validate_rigid_transform(np.asarray(record["visualization_matrix_m"]))["valid"])

    def test_accepted_visualization_is_the_formal_matrix(self):
        accepted=np.eye(4);accepted[0,3]=.0002
        c=self.diagnostic_candidate(status="refined_valid",matrix=accepted,proposals=[(accepted,True,[])])
        c["failure_reasons"]=[]
        record=SCRIPT.select_visualization_record(c)
        np.testing.assert_array_equal(record["visualization_matrix_m"],accepted)
        self.assertEqual(record["visualization_matrix_role"],"accepted_refinement")

    def test_visualization_labels_and_titles_distinguish_all_statuses(self):
        expected={"refined_valid":"PASS — accepted refinement","no_safe_refinement":"REJECTED — diagnostic only",
            "refinement_failed":"FAILED — diagnostic only"}
        for status,label in expected.items():
            c={**self.diagnostic_candidate(status=status),**{"visualization_source_kind":"proposed_matrix_m",
                "visualization_source_algorithm":"gicp","visualization_source_level":2,"visualization_source_iteration":3,
                "visualization_rejection_reasons":["reason"],"visualization_from_parent_rotation_deg":.1,
                "visualization_from_parent_translation_m":.0004}}
            self.assertEqual(_stage4_status_presentation(status)[0],label)
            title=_stage4_candidate_title(c);self.assertIn(label,title);self.assertIn("source=gicp L2 iter 3",title);self.assertIn("reason=reason",title)

if __name__=="__main__":unittest.main()

