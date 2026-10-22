from pathlib import Path
import zipfile
ROOT=Path(__file__).resolve().parents[1]
target=ROOT/'output/G1_left_tri_rail_v0_CONCEPT.zip'
files=[ROOT/'README.md',ROOT/'build.ps1']
for folder in ['config','scripts','sources','output/v0']:
    files += [p for p in (ROOT/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts]
files += [ROOT/'output/bracket_measurements.json',ROOT/'output/head_inspection.png']
with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED) as archive:
    for p in files:
        archive.write(p,p.relative_to(ROOT).as_posix())
with zipfile.ZipFile(target) as archive:
    assert archive.testzip() is None
    for required in ['output/v0/assembly_CONCEPT.step','output/v0/cad_preview.png','sources/head_link.STL','config/mount_v0.json']:
        assert required in archive.namelist()
print(f'Validated {len(files)} entries; {target.stat().st_size} bytes')
