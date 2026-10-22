# G1 camera CAD and button simulation backup

Private preservation of selected local G1 development work, prepared on 2026-10-23. This snapshot contains 60 source/output files from local prepared commit `b197194a1ee5903fadf2b61a5369b4afb719f00a`, including the six camera-mount STEP/STL outputs first saved in `161090e72e86a324802615fd9594d1cf16f65668`.

## Contents

- `Perception/real/camera_mount/g1_left_head`: latest head mount and camera clamp CAD, generator scripts/configuration, design reports and source attribution.
- `Manipulation/push_button`: the local camera handoff/vision additions, RGB-only, dual-camera and VLM simulation scripts, associated tests and recorded result documents.
- `Common/network/G1_WIRELESS_AOYAMA_DRAFT.md`: the existing sanitized wireless procedure draft.

## Provenance and restoration

The source repository is `https://github.com/airlayout/G1_Hackason`, with upstream snapshot `33b309665aa513c84cd22e6710753faf889f8cbe`. Original relative paths and source attribution are preserved. To reconstruct the full development context, obtain the source repository separately at the recorded revision and overlay the selected paths. This small backup does not contain all upstream runtime dependencies or third-party geometry; their recorded references remain in the generator source provenance files.

`BACKUP-MANIFEST.json` records copied file sizes/SHA256 and the source commit's Git blob identifiers. `BACKUP-PROVENANCE.json` records source revisions and exclusions. Existing result documents describe historical runs and do not indicate new testing or hardware validation.

## Exclusions

This repository begins with a new root commit, so the unrelated upstream parent history is not sent. Venue/private photos, videos, point clouds, maps, weights/checkpoints, dependency environments, downloaded packages, authentication material and printer/connection records are excluded. The full original local repository and its prepared commit remain intact on the PC.

No experiment, test, simulation, training, printer or robot connection was executed during this backup. Physical fit and print readiness have not been revalidated.
