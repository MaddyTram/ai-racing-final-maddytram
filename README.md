# ai-racing-final-maddytram
# AI Racing League final project

## Author and members
Maddy Tram (Solo)

## Problem statement
The goal of this project is to configure, tune, and execute an autonomous racing controller within the TORCS simulation environment. By interfacing Python client code with the TORCS server over socket connections, the autonomous driver navigates track curvature, adjusts speed parameters, and optimizes lap stability.

## Environment and setup
- OS: Windows 11. The race harness/tools start `wtorcs.exe` and are Windows-only; the driver itself is plain Python.
- Python: 3.14 (or Python 3.x standard environment).
- TORCS and challenge kit: TORCS with the Simulated Car Racing server (`scr_server`), installed at `C:\torcs\torcs`. (TORCS simulator is separate from this repository).
- Required local files: `torcs_jm_par.py` and `snakeoil3_jm2.py`.
- External dependencies: Listed in `requirements.txt` (`numpy`, `gym`). Install via:
  ```bash
  pip install -r requirements.txt
## RUN
From the gym_torcs folder, start the TORCS simulator application first, wait for the track server to start, and then launch the Python driver client script.

Windows command: py torcs_jm_par.py --host localhost --port 3001 --id SCR --steps 100000 --episodes 1 --stage 3
macOS/Linux command: python3 torcs_jm_par.py --host localhost --port 3001 --id SCR --steps 100000 --episodes 1 --stage 3

Note: The TORCS simulator application must be started first before launching the client script to establish the socket connection
## Results
Real Baseline: 'snakeoil3_gym.py'
Experiment: 'torcs_jm_par.py'
Track/setting: Corkscrew, car1-ow1, one lap from standing start
Completed Lap Time: 2:11:30, no damage
## Enhancements
1. **Braking & Cornering Stability:** Increased braking force so the car successfully negotiates sharp corners without running wide.
2. **Turn-In Speed Control:** Reduced entry speeds specifically for the chicane and the kink to maintain control through sharp direction changes.
3. **Throttle Modulation:** Smoothed throttle output to eliminate feathering/oscillation when cruising at target speeds or traveling at lower speeds.
4. **Top Speed Optimization:** Adjusted gear and speed parameters to allow the car to achieve a higher top speed on straightaways.
## Use of IBM Bob
- Ask. Bob's first task was to review the starter driver, analyze the driving logic, and help outline initial testing strategies.
- Plan. Bob proposed structured tuning plans and multi-perspective evaluations (acting as a race engineer and data analyst) to assess vehicle stability across test runs.
- Code. Bob proposed and generated adjustments for key vehicle enhancements, including increasing braking force for sharp corners, reducing entry speeds into the chicane and the kink to prevent running wide, smoothing throttle output to stop feathering at speed, and adjusting gear/speed parameters to reach a slightly higher top speed.
- Review. Bob served as a reviewer and second opinion on work in progress. Specific code adjustments proposed by Bob—such as significantly increasing speed when the car went off (due to incorrect brake balances) and other tuning adjustments that caused the car to oscillate or wiggle back and forth on the straights and coming out of corners—were carefully tested, rejected, and recorded like every other result.
## Demonstration
https://drive.google.com/file/d/1FG2vM-FbjpMgzuQbUh4WmhBwosiO-aq5/view?usp=drive_link
## Limitations
Couldn't get the car to reach the top speeds I wanted of around 250km/h on the straights and like 100km/h in the corners. I also wanted the car to maintain it's speed from the straights going into that little hill and take a better racing line
## Credits and licensing
- Kit/source attribution: `snakeoil3_gym.py` is the baseline driver supplied with the challenge kit, unmodified. The client library framework (`Client`, `ServerState`, `DriverAction`) used by `torcs_jm_par.py` is based on `snakeoil` by Chris X Edwards for the Simulated Car Racing server.
- Redistribution terms: TORCS is a separate open-source project (GNU GPL v2 or later) and is not included. All starter kit code remains attributed to its original authors, and ownership of the provided base code is not claimed[cite: 8]. Custom logic and parameter adjustments in `torcs_jm_par.py` represent the submission's original work.
