"""Reuse the validated bridge linker environment, including ROS 1/2 soname choices."""
import os
import shlex
import subprocess
from pathlib import Path

root=Path(os.environ['SAWYER_ROOT'])
build=root/'bridge_ws/build_conda/ros1_bridge'
command=shlex.split((build/'CMakeFiles/dynamic_bridge.dir/link.txt').read_text())
obj=root/'.runtime/native_build_conda/CMakeFiles/check_conversion.dir/check_conversion.cpp.o'
binary=root/'.runtime/native_build_conda/check_conversion'
replacements=0
for i,arg in enumerate(command):
    if arg.endswith('src/dynamic_bridge.cpp.o'):
        command[i]=str(obj)
        replacements+=1
    elif arg.startswith('-Wl,--dependency-file='):
        command[i]='-Wl,--dependency-file='+str(root/'.runtime/native_build_conda/link.d')
    elif i and command[i-1]=='-o':
        command[i]=str(binary)
assert replacements==1,'Unrecognized upstream bridge link command'
# Argument list execution avoids shell interpretation; no build files are edited.
subprocess.run(command,cwd=build,check=True)
