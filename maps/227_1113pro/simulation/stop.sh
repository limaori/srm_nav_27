#!/usr/bin/env bash
set -eo pipefail
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 - "$SOURCE_DIR/../../.." <<'PY'
import os
from pathlib import Path
import signal
import subprocess
import sys
root=Path(sys.argv[1]).resolve()
marker='sim_config_file:='+str(root/'log_diag/sim_tdt_smoother_20261117/sim.yaml')
for line in subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines():
    parts=line.strip().split(None,1)
    if len(parts)!=2:
        continue
    pid,command=parts
    if command.startswith('/usr/bin/python3 /opt/ros/humble/bin/ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py ') and marker in command:
        print('Stopping this simulation launch:',pid)
        os.kill(int(pid),signal.SIGINT)
PY
