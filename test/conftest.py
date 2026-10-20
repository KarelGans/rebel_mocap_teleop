"""Make the package importable as `rebel_demo` from the repo root (python3 -m pytest test/), without a build."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
