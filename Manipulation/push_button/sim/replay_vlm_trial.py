#!/usr/bin/env python3
"""保存済み視覚判断の再現、または固定目標の負例を実行する。新規VLM評価ではない。"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from run_vlm_closed_loop import initial, action, video, write_json


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("replay", "frozen-target-baseline"), default="replay")
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--model", required=True)
    parser.add_argument("--offset", nargs=3, type=float)
    args = parser.parse_args()
    source_config = json.loads((args.source / "scene.json").read_text(encoding="utf-8"))
    offset = args.offset if args.offset is not None else source_config["offset"]
    initial(args.out, args.model, offset)
    if args.mode == "frozen-target-baseline":
        # 元シーンでRGB-D推定した座標を凍結。変動後の画像を見ない比較専用。
        (args.out / "perceived_target.json").write_bytes((args.source / "perceived_target.json").read_bytes())
        decisions = [{"phase": "approach", "occluded_target_memory": True},
                     {"phase": "press", "occluded_target_memory": True, "press_extension_m": .0007}]
    else:
        decisions = [json.loads(p.read_text(encoding="utf-8"))
                     for p in sorted(args.source.glob("decision_*.json"))]
    for idx, saved in enumerate(decisions):
        obs = json.loads((args.out / "observation.json").read_text(encoding="utf-8"))
        decision = dict(saved)
        decision["original_model"] = decision.pop("model", None)
        decision["model"] = None
        decision["controller"] = args.mode
        decision["observation_route"] = "Recorded decision replay; no new model image call"
        decision["observation_sha256"] = obs["rgb_sha256"]
        path = args.out / f"decision_{idx:02d}.json"
        write_json(path, decision)
        action(args.out, path)
    video(args.out)


if __name__ == "__main__":
    main()
