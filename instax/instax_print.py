#!/usr/bin/env python3
#  Copyright (c) 2023. Honda Research Institute Japan. All rights reserved.

# Alternative approach to starting Python node under ROS that is using Python 2.X
# environment. For Python nodes that require Python 3 or higher,
# this approach allows to launch the node using a native Python 3 interpreter or
# a Virtual Environment interpreter even if the the initial process was started using Python 2.X.
# In such a case, the process is started as a Python 3.X subprocess,
# whilst original master process running under Python 2.X is just waiting for that child process to exit
# without consuming any resources.
import os
import subprocess
import sys
import logging
from time import sleep


# Check if we are under virtual environment.
# If not, run under it

def is_venv():
    return (hasattr(sys, 'real_prefix') or
            (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix))

# Define the argument to look for
arg='--venv'

# Remove the script name
args = sys.argv[1:]

# Get the path to the virtual environment
path_to_venv = args[args.index(arg) + 1].rstrip('/')

if not is_venv():
    # Create python bin path
    python_bin = path_to_venv + "/bin/python"

else:
    # Otherwise, just run with the current interpreter
    python_bin = sys.executable

# Get the actual executable file
dirname, filename = os.path.split(os.path.abspath(__file__))
script_file = dirname + "/print.py"

try:
    # Execute the file
    
    s = subprocess.Popen([python_bin, script_file] + args)
    # Sleep until the subprocess is running
    return_node = s.wait()
    exit(return_node)

except KeyboardInterrupt:

    while s.poll() is None:
        sleep(5)

    return_node = s.wait()
    exit(return_node)