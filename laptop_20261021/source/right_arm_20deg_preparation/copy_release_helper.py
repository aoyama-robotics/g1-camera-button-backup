from pathlib import Path
import sys
root=Path(__file__).resolve().parent
s=(root.parent/'release_ai_for_supported_vr.py').read_text(encoding='utf-8')
s=s.replace("Path('supported_vr_release_result.json')", "Path(__file__).with_name("+repr(sys.argv[1] if len(sys.argv)>1 else 'release_after_battery.json')+")")
(root/'release_ai_preparation.py').write_text(s,encoding='utf-8')
