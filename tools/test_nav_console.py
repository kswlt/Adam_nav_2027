"""Device-free CLI checks: mapping/navigation must never enable physical output."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SCRIPT=Path(__file__).parent/'deployment/20261021/nav_console.py'


class ConsoleTest(unittest.TestCase):
    def run_ui(self,*args):
        result=subprocess.run([sys.executable,str(SCRIPT),'--dry-run',*args],capture_output=True,text=True,check=True)
        commands=[]
        for line in result.stdout.splitlines():
            if line.startswith('[预演，不执行] '):commands.append(json.loads(line.split('] ',1)[1]))
        return commands

    def test_mapping_does_not_enable_serial(self):
        commands=self.run_ui('--action','mapping')
        starts=[c[3] for c in commands if len(c)>3 and c[2]=='start']
        self.assertEqual(starts,['driver','state','foxglove','capture','bag'])
        self.assertIn('physical',[c[3] for c in commands if c[2]=='stop'])

    def test_shadow_navigation_preserves_map_argument(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'map with spaces.json'
            path.write_text('{}')
            commands=self.run_ui('--action','navigation','--map-bundle',str(path),'--map-profile','local_site')
            nav=[c for c in commands if c[2:4]==['start','navigation']][0]
            self.assertEqual(nav[4:],[str(path),'local_site'])
            self.assertFalse(any(c[2:4]==['start','physical'] for c in commands))

    def test_stop_only_uses_scoped_manager(self):
        commands=self.run_ui('--action','stop')
        self.assertEqual(len(commands),9)
        self.assertTrue(all(c[2]=='stop' and c[1].endswith('vehicle_session.py') for c in commands))


if __name__=='__main__':unittest.main()
