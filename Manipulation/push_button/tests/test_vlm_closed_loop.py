"""押下の物理判定と、RGB-D推定による目標変更を検証する。"""
from __future__ import annotations
import sys
import unittest
import json
from unittest.mock import patch
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))
from run_vlm_closed_loop import physical_success, action, digest


class SuccessTest(unittest.TestCase):
    def setUp(self) -> None:
        self.metrics = {"max_stroke_m": .0015, "max_other_stroke_m": 0.,
                        "contact_at_threshold_steps": 50}

    def test_physical_success(self) -> None:
        self.assertTrue(physical_success(self.metrics))

    def test_travel_without_contact_rejected(self) -> None:
        self.metrics["contact_at_threshold_steps"] = 0
        self.assertFalse(physical_success(self.metrics))

    def test_too_brief_contact_rejected(self) -> None:
        self.metrics["contact_at_threshold_steps"] = 49
        self.assertFalse(physical_success(self.metrics))

    def test_undertravel_rejected(self) -> None:
        self.metrics["max_stroke_m"] = .001
        self.assertFalse(physical_success(self.metrics))

    def test_overtravel_rejected(self) -> None:
        self.metrics["max_stroke_m"] = .002
        self.assertFalse(physical_success(self.metrics))

    def test_wrong_button_rejected(self) -> None:
        self.metrics["max_other_stroke_m"] = .0002
        self.assertFalse(physical_success(self.metrics))


class ObservationBindingTest(unittest.TestCase):
    def test_changed_image_rejected_before_action(self) -> None:
        records = [json.dumps({"rgb": "obs.png", "rgb_sha256": "original"}),
                   json.dumps({"observation_sha256": "original", "phase": "press"})]
        with patch("run_vlm_closed_loop.load", return_value=({}, None, None, None)), \
             patch.object(Path, "read_text", side_effect=records), \
             patch("run_vlm_closed_loop.digest", return_value="changed"):
            with self.assertRaises(ValueError):
                action(Path("unused"), Path("decision.json"))


if __name__ == "__main__":
    unittest.main()
