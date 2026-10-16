# Submission Documentation

## 1. Submission Details
* **Primary Script:** `torcs_jm_par.py`
* **Baseline Status:** Original baseline (`snakeoil3_gym.py` v0.1) preserved; final driver optimized and tuned to v1.33.

## 2. Experimental Results & Performance Metrics

| Metric | Real Baseline (`snakeoil3_gym.py` v0.1) | Final Driver (`torcs_jm_par.py` v1.33) |
| :--- | :--- | :--- |
| **Best Lap Time (s)** | Crashed on lap 1 (No time) | **2:11:30** |
| **Target / Speed** | Aimed for 300 km/h (No braking) | Controlled entry speeds & optimized top speed, around 150km/h |
| **Off-Track / Damage** | Severe crash | **Zero damage** |
| **Completion Rate (%)** | 0% (Did not finish) | **100% (Completed)** |

## 3. Unavailable Kit Components & Setup Requirements
* **Physical Hardware/Sensors:** Physical telemetry or sensor hardware kits were unavailable; testing and execution were completed entirely within the TORCS simulation environment.
* **Simulator Integration:** Requires socket connectivity to a running TORCS server instance (`localhost:3001`).
