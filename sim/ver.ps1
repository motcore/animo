# Opens the T-rex simulation in a window (real time). Extra knobs pass on:
#   .\ver.ps1 freq=0.9 phase=135
& "$PSScriptRoot\.venv\Scripts\python.exe" "$PSScriptRoot\trex_sim.py" --view --set secs=40 @args
