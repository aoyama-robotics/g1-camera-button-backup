#!/usr/bin/env python3
"""Replay saved semantic decisions; this does not call a VLM."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
from run_dual_camera import init,act
from run_vlm_closed_loop import write_json


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--model",required=True)
    parser.add_argument("--camera-config",type=Path,required=True)
    args = parser.parse_args()
    first = json.loads((args.source/"obs_00.json").read_text(encoding="utf-8"))
    init(args.out,args.model,args.camera_config,math.degrees(first["waist_yaw_self_state_rad"]))
    for original in sorted(args.source.glob("decision_*.json")):
        decision = json.loads(original.read_text(encoding="utf-8"))
        observation = json.loads((args.out/"observation.json").read_text(encoding="utf-8"))
        decision.update(model=None,execution_mode="recorded_semantic_decision_replay_no_VLM",
                        original_model=decision.get("model"),
                        observation_sha256=observation["pair_sha256"])
        destination = args.out/original.name
        write_json(destination,decision)
        act(args.out,destination)
    metrics = json.loads((args.out/"metrics.json").read_text(encoding="utf-8"))
    write_json(args.out/"replay_result.json",{
        "execution_mode":"recorded_semantic_decision_replay_no_VLM",
        "source":str(args.source.resolve()),"metrics":metrics})
    print(json.dumps(metrics))
    if not metrics.get("success"):
        raise SystemExit(1)


if __name__=="__main__":
    main()
