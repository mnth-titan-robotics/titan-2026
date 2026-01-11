import sys
import subprocess
import tomllib

with open("pyproject.toml", "rb") as f:
  toml = tomllib.load(f)
robotpy_version = toml["tool"]["robotpy"]["robotpy_version"]

# Get the path to the python interpreter executing this script
python_path = sys.executable

# Execute  
subprocess.run([
  python_path,
  "-m", "pip", "install",
  # Ensure the specific version of robotpy is installed 
  "--upgrade", f"robotpy=={robotpy_version}",
  # Install certifi, but don't worry about upgrading to a specific version 
  "certifi"])
