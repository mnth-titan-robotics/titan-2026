import sys
import os

# Add the generated folder to sys.path
generated_path = os.path.dirname(__file__)
if generated_path not in sys.path:
    sys.path.insert(0, generated_path)