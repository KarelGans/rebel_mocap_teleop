"""Lets `python3 -m pytest test/` import the package from this source tree (no build or sourcing needed)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
