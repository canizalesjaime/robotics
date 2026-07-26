import os
import sys

workspace = "/workspaces/homebot"
backend_path = os.path.join(workspace, "backend")

if backend_path not in sys.path:
    sys.path.append(backend_path)