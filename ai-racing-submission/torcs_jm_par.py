import getopt
import json
import os
import socket
import sys
import time

PI = 3.14159265359

data_size = 2**17

ophelp = 'Options:\n'
ophelp += ' --host, -H <host>    TORCS server host. [localhost]\n'
ophelp += ' --port, -p <port>    TORCS port. [3001]\n'
ophelp += ' --id, -i <id>        ID for server. [SCR]\n'
ophelp += (
    ' --steps, -m <#>      Maximum simulation steps. 1 sec ~ 50 steps.'
    ' [100000]\n'
)
ophelp += ' --episodes, -e <#>   Maximum learning episodes. [1]\n'
ophelp += (
    ' --track, -t <track>  Your name for this track. Used for learning.'
    ' [unknown]\n'
)
ophelp += ' --stage, -s <#>      0=warm up, 1=qualifying, 2=race, 3=unknown. [3]\n'
ophelp += (
    ' --parameters, -f <#>  Input parameters of target_speed : int,'
    ' steer_gain : int, centering_gain : float, brake_threshold : float,'
    ' 6*[gear_speeds : int] (dont put brackets in the input tho),'
    ' enable_traction_control : bool. [1]\n'
)
ophelp += ' --debug, -d          Output full telemetry.\n'
ophelp += ' --help, -h           Show this help.\n'
ophelp += ' --version, -v        Show current version.'
usage = 'Usage: %s [ophelp [optargs]] \n' % sys.argv[0]
usage = usage + ophelp
version = '20130505-2'

# =============================================
# MODULAR DRIVE LOGIC WITH USER PARAMETERS  #
# =============================================

import math

# ================= USER CONFIGURABLE PARAMETERS =================
# -------- REQUIRED FOR PRACTICE MODE - CAN CHANGE BUT DO NOT REMOVE --------
TARGET_SPEED = 300  # Target speed in km/h. Increasing this makes the car go faster but may reduce stability.
STEER_GAIN = 54  # Steering sensitivity. Higher values make the car turn more aggressively.
CENTERING_GAIN = (
    0.18  # How strongly the car corrects its position toward the center of the track.
)
BRAKE_THRESHOLD = 0.15  # Angle threshold for braking. Lower values brake earlier.
GEAR_SPEEDS = [0, 40, 75, 130, 165, 210]  # Speed thresholds for gear shifting.
ENABLE_TRACTION_CONTROL = True  # Toggle traction control system.
# --------- END OF REQUIRED ---------


# ================= HELPER FUNCTIONS =================
def calculate_steering(S):
    speed = S['speedX']
    angle_abs = abs(S['angle'])
    track_pos = S['trackPos']
    track = S['track']

    # --- straight-stability scalar -------------------------------------------------
    # Suppresses gain near angle=0 at speed to prevent straight-line oscillation.
    # Reaches 1.0 (full gain) once angle >= 0.15 rad.
    angle_component = min(1.0, angle_abs / 0.15)
    speed_component = max(0.0, 1.0 - speed / 200.0)
    straight_scalar = max(angle_component, speed_component)
    straight_scalar = max(0.18, straight_scalar)

    # --- corner boost -------------------------------------------------------------
    # Fires at >= 0.12 rad — medium and sharp turns only.
    if angle_abs >= 0.12:
        corner_boost = (
            1.0 + (angle_abs - 0.12) / 0.20 * 0.9
        )  # up to x1.9 at 0.32 rad
        corner_boost = min(1.9, corner_boost)
    else:
        corner_boost = 1.0

    # --- kink geometry detector ---------------------------------------------------
    wide_left = track[3]  # -7 deg
    wide_right = track[15]  # +7 deg
    ahead = track[9]
    wide_closing = max(0.0, ahead - min(wide_left, wide_right))
    kink_factor = min(1.0, max(0.0, (wide_closing - 20.0) / 35.0))

    # --- mid-speed straight damper ------------------------------------------------
    # The 80-120 km/h window is where gear-change acceleration happens on straights.
    # With small angle the existing straight_scalar still leaves enough gain to
    # sustain oscillation. This damper cuts both gains by 60% in that window.
    #
    # Conditions (all must hold — no cut during genuine corners or kinks):
    #   80 < speed < 120 km/h
    #   angle_abs < 0.08 rad  (~4.6 deg — clearly not in a corner)
    #   kink_factor == 0       — kink approach never dampened
    #
    # Smooth window edges: ramp in over 80-90 km/h, ramp out over 110-120 km/h
    # so there is no step-change at either boundary.
    if kink_factor == 0.0 and angle_abs < 0.08 and 80.0 < speed < 120.0:
        ramp_in = min(1.0, (speed - 80.0) / 10.0)  # 0.0 at 80, 1.0 at 90
        ramp_out = min(1.0, (120.0 - speed) / 10.0)  # 1.0 at 110, 0.0 at 120
        window = min(ramp_in, ramp_out)  # plateau 1.0 from 90-110
        mid_speed_damper = 1.0 - 0.60 * window  # 60% cut at plateau
    else:
        mid_speed_damper = 1.0

    # --- kink entry steer rate limiter --------------------------------------------
    # When kink_factor is active we want maximum rotation speed toward the apex,
    # so the scalar is 1.0 (no drag). Only applied on non-kink corners to smooth
    # entry for chicane/90-degree turns where snap oversteer is a risk.
    kink_entry_scalar = 1.0 if kink_factor > 0.0 else 1.0

    # --- effective gains ----------------------------------------------------------
    effective_gain = (
        STEER_GAIN
        * straight_scalar
        * corner_boost
        * (1.0 + kink_factor * 1.8)
        * mid_speed_damper
        * kink_entry_scalar
    )

    speed_factor = min(1.0, speed / 80.0)
    active_center = (
        CENTERING_GAIN * speed_factor * straight_scalar * mid_speed_damper
    )

    # --- kink apex hold -----------------------------------------------------------
    # Once kink_factor fires, hold the apex pull until the car is genuinely
    # straight (angle_abs < 0.05). This prevents the centering urgency from
    # collapsing mid-corner just because wide_closing shrinks as the car turns.
    if kink_factor > 0.0 or angle_abs > 0.05:
        kink_hold = max(kink_factor, min(1.0, (angle_abs - 0.02) / 0.03))
    else:
        kink_hold = 0.0

    # --- apex-biased centering target ---------------------------------------------
    if angle_abs > 0.06:
        corner_sign_stable = 1.0 if S['angle'] > 0.0 else -1.0
        apex_strength = min(
            1.0, (angle_abs - 0.06) / 0.10
        )  # 0→1 over 0.06–0.16 rad
        apex_offset = -corner_sign_stable * 0.45 * apex_strength
    elif kink_hold > 0.0 and angle_abs > 0.03:
        link_sign = 1.0 if track_pos < 0.0 else -1.0
        link_fade = min(1.0, (angle_abs - 0.03) / 0.07)  # 0→1 over 0.03–0.10 rad
        apex_offset = link_sign * 0.25 * kink_hold * link_fade
    else:
        apex_offset = 0.0

    pos_error = track_pos - apex_offset

    # trackPos urgency with kink apex pull held through corner exit.
    pos_abs = abs(pos_error)
    speed_urgency = max(0.6, 1.0 - speed / 280.0)
    kink_urgency = 1.0 + kink_hold * 3.0

    if pos_abs > 0.5:
        center_urgency = (1.0 + 2.8 * speed_urgency) * kink_urgency
    elif pos_abs > 0.30:
        center_urgency = (1.0 + 1.4 * speed_urgency) * kink_urgency
    elif pos_abs > 0.15:
        center_urgency = (1.0 + 0.6 * speed_urgency) * kink_urgency
    else:
        center_urgency = 1.0 * kink_urgency

    steer = (S['angle'] * effective_gain / math.pi) - (
        pos_error * active_center * center_urgency
    )

    # Angle boost for deep corners — fires at >= 0.15 rad.
    if angle_abs > 0.15:
        boost = 1.0 + (angle_abs / math.pi) * 2.4
        steer = max(-1, min(1, steer * boost))

    return max(-1, min(1, steer))


def _corner_safe_speed(track):
    """Derive a safe target speed from the full forward sensor arc (indices 4-14)."""
    min_sensor = min(track[4:15])
    ahead = track[9]

    left_arc = track[4:9]
    right_arc = track[10:15]
    asymmetry = abs(min(left_arc) - min(right_arc))

    wide_left = track[3]  # -7°
    wide_right = track[15]  # +7°
    wide_min = min(wide_left, wide_right)

    severity = max(0.0, ahead - min_sensor)
    wide_severity = max(0.0, ahead - wide_min) * 0.5

    effective_severity = max(severity, wide_severity)

    tight_factor = min(0.7, effective_severity / 160.0)
    w_min = 0.35 + tight_factor
    w_ahead = max(0.05, 1.0 - w_min)
    corridor = min_sensor * w_min + ahead * w_ahead
    corridor = max(0.0, min(200.0, corridor))

    MIN_SAFE_SPEED = 55.0
    scale = (corridor - 10.0) / (200.0 - 10.0)
    scale = max(0.0, min(1.0, scale))
    raw_safe = MIN_SAFE_SPEED + scale * (TARGET_SPEED - MIN_SAFE_SPEED)

    if effective_severity > 60.0 and asymmetry > 70.0:
        asym_penalty = min(40.0, (asymmetry - 70.0) * 0.42)
    else:
        asym_penalty = 0.0

    if effective_severity < 40.0:
        lookahead = effective_severity * 0.025
    elif effective_severity < 60.0:
        lookahead = 1.0 + (effective_severity - 40.0) * 0.25
    else:
        lookahead = min(50.0, 6.0 + (effective_severity - 60.0) * 0.28)

    kink_penalty = 0.0
    if wide_severity > 30.0 and ahead > 150.0 and severity < effective_severity:
        kink_penalty = 108.0

    return max(
        MIN_SAFE_SPEED, raw_safe - asym_penalty - lookahead - kink_penalty
    )

def apply_brakes(S):
    """Corner-entry braking over the full forward sensor arc.

    Three layered mechanisms:
      1. Speed-excess curved brake — heavier on straight approach, capped
         low once the car is turning so front tyres retain steering grip.
      2. Corner-asymmetry early brake — fires before corner entry.
      3. Proximity wall guard — emergency only, narrow arc.

    Key rule: when the car is actively turning (|angle| > 0.10 rad) the
    global brake cap is limited to 0.20 so the front wheels never lock and
    steering authority is preserved through the corner.
    """
    track  = S['track']
    speed  = S['speedX']
    angle  = abs(S['angle'])

    safe_speed = _corner_safe_speed(track)

    # --- mechanism 1: speed-excess curved brake ------------------------------
    # Use same wide-sensor effective_severity for dead_band so kink braking
    # starts on the straight at the same point safe_speed begins dropping.
    speed_excess  = speed - safe_speed
    min_sensor    = min(track[4:15])
    wide_min_b    = min(track[3], track[15])
    ahead_b       = track[9]
    sev_narrow    = max(0.0, ahead_b - min_sensor)
    sev_wide      = max(0.0, ahead_b - wide_min_b) * 0.6   # raised 0.5->0.6: dead_band shrinks earlier on kink approach
    eff_sev       = max(sev_narrow, sev_wide)
    dead_band     = max(3.0, 7.0 - eff_sev * 0.055)   # earlier braking on straight

    if speed_excess > dead_band:
        ratio = min(1.0, (speed_excess - dead_band) / 45.0)   # sharper ramp: full brake at 45+ km/h over
        brake_force = min(0.35, ratio ** 1.4)                  # softer exponent: builds faster at low excess
    else:
        brake_force = 0.0

    # --- mechanism 2: asymmetry brake
    left_min  = min(track[4:9])
    right_min = min(track[10:15])
    asymmetry = abs(left_min - right_min)
    if asymmetry > 120.0 and speed > 140.0:
        asym_brake  = min(0.12, (asymmetry - 120.0) / 80.0 * 0.12)
        speed_scale = min(1.0, (speed - 140.0) / 80.0)
        brake_force = max(brake_force, asym_brake * speed_scale)

    # --- mechanism 3: proximity wall guard (narrow ±1.7° arc only) ----------
    # Thresholds raised so this only fires for genuine imminent-wall emergencies,
    # not normal kink geometry where walls naturally appear closer on one side.
    min_forward = min(track[6:13])
    if min_forward < 3.0:
        brake_force = max(brake_force, 0.35)
    elif min_forward < 6.0:
        proximity_brake = 0.08 + 0.27 * (1.0 - (min_forward - 3.0) / 3.0)
        brake_force = max(brake_force, proximity_brake)

    # --- turning cap: zero brakes whenever steering is active ------------------
    # Any meaningful yaw angle means the car is turning — release brakes fully
    # so front tyres have 100% traction for cornering, not braking.
    if angle > 0.05:
        return 0.0

    return min(0.35, brake_force)   # global cap 0.35 — car never stalls at 0 km/h

def calculate_throttle(S, R):
    """Speed-scaled throttle differentiated by corner severity tier.

    Slight corners: full throttle up to safe_speed, no cap.
    Medium corners: mild steer cap only for extreme lock.
    Tight corners (chicane / 90°): both asymmetry and steer caps apply.
    """
    track = S['track']
    safe_speed = _corner_safe_speed(track)
    speed = S['speedX']

    # Kill throttle only for heavy brake application.
    # Raised threshold means light/medium braking through chicane keeps
    # partial throttle — car stays alive and can steer back to the line.
    brake_val = apply_brakes(S)
    if brake_val > 0.35:
        return 0.0
    elif brake_val > 0.25:
        # Partial throttle during moderate braking: 0→0.25 over 0.25→0.35
        return max(0.0, 0.25 * (1.0 - (brake_val - 0.25) / 0.10))

    # --- kink / corner throttle cap -----------------------------------------------
    # Two-part cap that covers both approach (kink geometry sensor) and mid-corner
    # (angle-based), so throttle stays light from initial turn-in until the car
    # is genuinely straight again.
    #
    # Part A — geometry-based (approach): uses same ramp as steering kink_factor.
    # Caps throttle as the ±7° sensors close on approach.
    wide_left_t = track[3]
    wide_right_t = track[15]
    ahead_t = track[9]
    wide_closing_t = max(0.0, ahead_t - min(wide_left_t, wide_right_t))
    kink_factor_t = min(1.0, max(0.0, (wide_closing_t - 20.0) / 35.0))

    # Part B — angle-based (mid-corner / exit): holds throttle light while the
    # car is still rotating. Scales from 0.40 at angle>=0.10 rad back to 1.0
    # at angle<=0.02 rad so acceleration builds smoothly as the car straightens.
    #   angle >= 0.10 rad  → accel_angle_cap = 0.40  (full corner — light throttle)
    #   angle   0.02 rad   → accel_angle_cap = 1.0   (straight — full throttle)
    angle_abs_t = abs(S['angle'])

    if angle_abs_t >= 0.10:
        accel_angle_cap = 0.40
    elif angle_abs_t > 0.02:
        t = (angle_abs_t - 0.02) / 0.08  # 0 at 0.02 rad, 1 at 0.10 rad
        accel_angle_cap = 0.40 + (1.0 - 0.40) * (1.0 - t)  # 1.0 -> 0.40
    else:
        accel_angle_cap = 1.0

    # Combined: tightest of geometry cap and angle cap.
    if kink_factor_t > 0.0:
        geom_cap = 1.0 - kink_factor_t * 0.60  # 1.0 -> 0.40 on approach
    else:
        geom_cap = 1.0

    kink_throttle_cap = min(geom_cap, accel_angle_cap)

    # Classify corner by severity to set the right throttle cap
    min_sensor = min(track[4:15])
    ahead = track[9]
    severity = max(0.0, ahead - min_sensor)

    left_min = min(track[4:9])
    right_min = min(track[10:15])
    asymmetry = abs(left_min - right_min)

    # Asymmetry throttle cap — only for genuinely tight corners
    # slight/medium corners get 1.0 (no restriction from asymmetry)
    if severity > 120.0 and asymmetry > 100.0:
        corner_cap = 0.35
    elif severity > 120.0 and asymmetry > 60.0:
        corner_cap = 0.60
    else:
        corner_cap = 1.0

    # Steer magnitude cap — threshold raised so centering corrections on
    # straights (~0.05-0.35) never reduce throttle below 1.0.
    steer_mag = abs(R['steer'])
    if steer_mag > 0.50:
        steer_cap = 0.20
    elif steer_mag > 0.35:
        steer_cap = 0.50
    else:
        steer_cap = 1.0

    effective_cap = min(steer_cap, corner_cap)

    # --- straight-line full-throttle override ------------------------------------
    # When the car is clearly on a straight (small angle, near centre, no kink)
    # force accel=1.0 and bypass all sensor-derived caps.
    # This prevents uphill crests / short sensor readings from falsely triggering
    # corner_cap or steer_cap and dragging speed down on straight sections.
    # Conditions: angle < 0.06 rad, trackPos within ±0.4, no kink detected,
    #             no active brake, steer output small.
    on_straight_override = (
        angle_abs_t < 0.06
        and abs(S['trackPos']) < 0.4
        and kink_factor_t == 0.0
        and abs(R['steer']) < 0.30
    )
    if on_straight_override:
        return 1.0

    # Speed-scaled base throttle.
    # Flat-out until within 50 km/h of ceiling, then a steep concave taper.
    headroom = safe_speed - speed
    if headroom > 50.0:
        base_accel = 1.0
    elif headroom > 0.0:
        # Concave curve: at headroom=50 → 1.0, at headroom=20 → 0.93, at headroom=5 → 0.83
        taper = (headroom / 50.0) ** 0.4
        base_accel = max(0.0, min(1.0, taper))
    else:
        base_accel = max(0.0, R['accel'] - 0.35)
        
    return max(0.0, min(effective_cap, base_accel, kink_throttle_cap))
    
def shift_gears(S):
    gear = 1
    for i, speed in enumerate(GEAR_SPEEDS):
        if S['speedX'] > speed:
            gear = i + 1
    return min(gear, 6)


def traction_control(S, accel):
    if ENABLE_TRACTION_CONTROL:
        slip = (S['wheelSpinVel'][2] + S['wheelSpinVel'][3]) - (
            S['wheelSpinVel'][0] + S['wheelSpinVel'][1]
        )
        if slip > 15.0:
            # Only cut for severe wheelspin — high-speed rear-drive slip is
            # normal and should not cap straight-line throttle.
            # Threshold raised to 15; reduction capped at 0.04 per frame.
            accel -= min(0.04, (slip - 15.0) * 0.005)
    return max(0.0, accel)


def clip(v, lo, hi):
    if v < lo:
        return lo
    elif v > hi:
        return hi
    else:
        return v


def bargraph(x, mn, mx, w, c='X'):
    """Draws a simple asciiart bar graph. Very handy for

    visualizing what's going on with the data.
    x= Value from sensor, mn= minimum plottable value,
    mx= maximum plottable value, w= width of plot in chars,
    c= the character to plot with.
    """
    if not w:
        return ''  # No width!
    if x < mn:
        x = mn  # Clip to bounds.
    if x > mx:
        x = mx  # Clip to bounds.
    tx = mx - mn  # Total real units possible to show on graph.
    if tx <= 0:
        return 'backwards'  # Stupid bounds.
    upw = tx / float(w)  # X Units per output char width.
    if upw <= 0:
        return 'what?'  # Don't let this happen.
    negpu, pospu, negnonpu, posnonpu = 0, 0, 0, 0
    if mn < 0:  # Then there is a negative part to graph.
        if x < 0:  # And the plot is on the negative side.
            negpu = -x + min(0, mx)
            negnonpu = -mn + x
        else:  # Plot is on pos. Neg side is empty.
            negnonpu = -mn + min(0, mx)  # But still show some empty neg.
    if mx > 0:  # There is a positive part to the graph
        if x > 0:  # And the plot is on the positive side.
            pospu = x - max(0, mn)
            posnonpu = mx - x
        else:  # Plot is on neg. Pos side is empty.
            posnonpu = mx - max(0, mn)  # But still show some empty pos.
    nnc = int(negnonpu / upw) * '-'
    npc = int(negpu / upw) * c
    ppc = int(pospu / upw) * c
    pnc = int(posnonpu / upw) * '_'
    return '[%s]' % (nnc + npc + ppc + pnc)


class Client:

    def __init__(
        self, H=None, p=None, i=None, e=None, t=None, s=None, d=None, vision=False
    ):
        self.vision = vision
        self.host = 'localhost'
        self.port = 3001
        self.sid = 'SCR'
        self.maxEpisodes = 1  # "Maximum number of learning episodes to perform"
        self.trackname = 'unknown'
        self.stage = 3  # 0=Warm-up, 1=Qualifying 2=Race, 3=unknown <Default=3>
        self.debug = False
        self.maxSteps = 100000  # 50steps/second
        self.parse_the_command_line()
        if H:
            self.host = H
        if p:
            self.port = p
        if i:
            self.sid = i
        if e:
            self.maxEpisodes = e
        if t:
            self.trackname = t
        if s:
            self.stage = s
        if d:
            self.debug = d
        self.S = ServerState()
        self.R = DriverAction()
        self.setup_connection()

    def setup_connection(self):
        try:
            self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as emsg:
            print('Error: Could not create socket...')
            sys.exit(-1)
        self.so.settimeout(1)

        n_fail = 5
        while True:
            a = '-45 -19 -12 -7 -4 -2.5 -1.7 -1 -.5 0 .5 1 1.7 2.5 4 7 12 19 45'
            initmsg = '%s(init %s)' % (self.sid, a)
            try:
                self.so.sendto(initmsg.encode(), (self.host, self.port))
            except socket.error as emsg:
                sys.exit(-1)
            sockdata = str()
            try:
                sockdata, addr = self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error as emsg:
                print('Waiting for server on %d............' % self.port)
                print('Count Down : ' + str(n_fail))
                if n_fail < 0:
                    print('relaunch torcs')
                    os.system('pkill torcs')
                    time.sleep(1.0)
                    if self.vision is False:
                        os.system('torcs -nofuel -nodamage -nolaptime &')
                    else:
                        os.system('torcs -nofuel -nodamage -nolaptime -vision &')
                    time.sleep(1.0)
                    os.system('sh autostart.sh')
                    n_fail = 5
                n_fail -= 1

            identify = '***identified***'
            if identify in sockdata:
                print('Client connected on %d..............' % self.port)
                break

    def parse_the_command_line(self):
        try:
            (opts, args) = getopt.getopt(sys.argv[1:], 'H:p:i:m:e:t:s:f:dhv',
                       ['host=','port=','id=','steps=',
                        'episodes=','track=','stage=', "parameters=",
                        'debug','help','version'])
        except getopt.error as why:
            print('getopt error: %s\n%s' % (why, usage))
            sys.exit(-1)
        try:
            for opt in opts:
                if opt[0] == '-h' or opt[0] == '--help':
                    print(usage)
                    sys.exit(0)
                if opt[0] == '-d' or opt[0] == '--debug':
                    self.debug= True
                if opt[0] == '-H' or opt[0] == '--host':
                    self.host= opt[1]
                if opt[0] == '-i' or opt[0] == '--id':
                    self.sid= opt[1]
                if opt[0] == '-t' or opt[0] == '--track':
                    self.trackname= opt[1]
                if opt[0] == '-s' or opt[0] == '--stage':
                    self.stage= int(opt[1])
                ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
                if opt[0] == '-p' or opt[0] == '--port':
                    self.port= int(opt[1])
                ######## END OF REQUIRED #########
                if opt[0] == '-e' or opt[0] == '--episodes':
                    self.maxEpisodes= int(opt[1])
                if opt[0] == '-m' or opt[0] == '--steps':
                    self.maxSteps= int(opt[1])
                ######### REQUIRED FOR PRACTICE MODE - CAN CHANGE BUT DO NOT REMOVE #########
                if opt[0] == '-f' or opt[0] == '--parameters':
                    global TARGET_SPEED, STEER_GAIN, CENTERING_GAIN, BRAKE_THRESHOLD, GEAR_SPEEDS, ENABLE_TRACTION_CONTROL
                    json_object=json.loads(opt[1])
                    TARGET_SPEED = int(json_object['target_speed'])
                    STEER_GAIN = int(json_object['steer_gain'])
                    CENTERING_GAIN = float(json_object['centering_gain'])
                    BRAKE_THRESHOLD = float(json_object['brake_threshold'])
                    GEAR_SPEEDS = [int(x) for x in json_object['gear_thresholds']]
                    ENABLE_TRACTION_CONTROL = json_object['traction_control']
                    print(f"Running with parameters:\ntarget speed={TARGET_SPEED}\nsteer gain={STEER_GAIN}\ncentering gain={CENTERING_GAIN}\nbrake threshold={BRAKE_THRESHOLD}\ngear speeds={GEAR_SPEEDS}\ntraction control enabled={ENABLE_TRACTION_CONTROL}")
                ########## END OF REQUIRED #########
                if opt[0] == '-v' or opt[0] == '--version':
                    print('%s %s' % (sys.argv[0], version))
                    sys.exit(0)
        except ValueError as why:
            print('Bad parameter \'%s\' for option %s: %s\n%s' % (
                                       opt[1], opt[0], why, usage))
            sys.exit(-1)
        if len(args) > 0:
            print('Superflous input? %s\n%s' % (', '.join(args), usage))
            sys.exit(-1)

    ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
    def get_servers_input(self):
        '''Server's input is stored in a ServerState object'''
        if not self.so: return
        sockdata= str()

        while True:
            try:
                sockdata,addr= self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error as emsg:
                print('.', end=' ')
            if '***identified***' in sockdata:
                print("Client connected on %d.............." % self.port)
                continue
            elif '***shutdown***' in sockdata:
                print((("Server has stopped the race on %d. "+
                        "You were in %d place.") %
                        (self.port,self.S.d['racePos'])))
                self.shutdown()
                return
            elif '***restart***' in sockdata:
                print("Server has restarted the race on %d." % self.port)
                self.shutdown()
                return
            elif not sockdata: # Empty?
                continue       # Try again.
            else:
                self.S.parse_server_str(sockdata)
                if self.debug:
                    sys.stderr.write("\x1b[2J\x1b[H") # Clear for steady output.
                    print(self.S)
                break # Can now return from this function.
    ########## END OF REQUIRED #########

    def respond_to_server(self):
        if not self.so: return
        try:
            message = repr(self.R)
            self.so.sendto(message.encode(), (self.host, self.port))
        except socket.error as emsg:
            print("Error sending to server: %s Message %s" % (emsg[1],str(emsg[0])))
            sys.exit(-1)
        if self.debug: print(self.R.fancyout())

    def shutdown(self):
        if not self.so: return
        print(("Race terminated or %d steps elapsed. Shutting down %d."
               % (self.maxSteps,self.port)))
        self.so.close()
        self.so = None

class ServerState():
    '''What the server is reporting right now.'''
    def __init__(self):
        self.servstr= str()
        self.d= dict()

    def parse_server_str(self, server_string):
        '''Parse the server string.'''
        self.servstr= server_string.strip()[:-1]
        sslisted= self.servstr.strip().lstrip('(').rstrip(')').split(')(')
        for i in sslisted:
            w= i.split(' ')
            self.d[w[0]]= destringify(w[1:])

    def __repr__(self):
        return self.fancyout()
        out= str()
        for k in sorted(self.d):
            strout= str(self.d[k])
            if type(self.d[k]) is list:
                strlist= [str(i) for i in self.d[k]]
                strout= ', '.join(strlist)
            out+= "%s: %s\n" % (k,strout)
        return out

    def fancyout(self):
        '''Specialty output for useful ServerState monitoring.'''
        out= str()
        sensors= [ # Select the ones you want in the order you want them.
        'stucktimer',
        'fuel',
        'distRaced',
        'distFromStart',
        'opponents',
        'wheelSpinVel',
        'z',
        'speedZ',
        'speedY',
        'speedX',
        'targetSpeed',
        'rpm',
        'skid',
        'slip',
        'track',
        'trackPos',
        'angle',
        ]

        for k in sensors:
            if type(self.d.get(k)) is list: # Handle list type data.
                if k == 'track': # Nice display for track sensors.
                    strout= str()
                    raw_tsens= ['%.1f'%x for x in self.d['track']]
                    strout+= ' '.join(raw_tsens[:9])+'_'+raw_tsens[9]+'_'+' '.join(raw_tsens[10:])
                elif k == 'opponents': # Nice display for opponent sensors.
                    strout= str()
                    for osensor in self.d['opponents']:
                        if   osensor >190: oc= '_'
                        elif osensor > 90: oc= '.'
                        elif osensor > 39: oc= chr(int(osensor/2)+97-19)
                        elif osensor > 13: oc= chr(int(osensor)+65-13)
                        elif osensor >  3: oc= chr(int(osensor)+48-3)
                        else: oc= '?'
                        strout+= oc
                    strout= ' -> '+strout[:18] + ' ' + strout[18:]+' <-'
                else:
                    strlist= [str(i) for i in self.d[k]]
                    strout= ', '.join(strlist)
            else: # Not a list type of value.
                if k == 'gear': # This is redundant now since it's part of RPM.
                    gs= '_._._._._._._._._'
                    p= int(self.d['gear']) * 2 + 2  # Position
                    l= '%d'%self.d['gear'] # Label
                    if l=='-1': l= 'R'
                    if l=='0':  l= 'N'
                    strout= gs[:p]+ '(%s)'%l + gs[p+3:]
                elif k == 'damage':
                    strout= '%6.0f %s' % (self.d[k], bargraph(self.d[k],0,10000,50,'~'))
                elif k == 'fuel':
                    strout= '%6.0f %s' % (self.d[k], bargraph(self.d[k],0,100,50,'f'))
                elif k == 'speedX':
                    cx= 'X'
                    if self.d[k]<0: cx= 'R'
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k],-30,300,50,cx))
                elif k == 'speedY': # This gets reversed for display to make sense.
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k]*-1,-25,25,50,'Y'))
                elif k == 'speedZ':
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k],-13,13,50,'Z'))
                elif k == 'z':
                    strout= '%6.3f %s' % (self.d[k], bargraph(self.d[k],.3,.5,50,'z'))
                elif k == 'trackPos': # This gets reversed for display to make sense.
                    cx='<'
                    if self.d[k]<0: cx= '>'
                    strout= '%6.3f %s' % (self.d[k], bargraph(self.d[k]*-1,-1,1,50,cx))
                elif k == 'stucktimer':
                    if self.d[k]:
                        strout= '%3d %s' % (self.d[k], bargraph(self.d[k],0,300,50,"'"))
                    else: strout= 'Not stuck!'
                elif k == 'rpm':
                    g= self.d['gear']
                    if g < 0:
                        g= 'R'
                    else:
                        g= '%1d'% g
                    strout= bargraph(self.d[k],0,10000,50,g)
                elif k == 'angle':
                    asyms= [
                          "  !  ", ".|'  ", "./'  ", "_.-  ", ".--  ", "..-  ",
                          "---  ", ".__  ", "-._  ", "'-.  ", "'\.  ", "'|.  ",
                          "  |  ", "  .|'", "  ./'", "  .-'", "  _.-", "  __.",
                          "  ---", "  --.", "  -._", "  -..", "  '\.", "  '|."  ]
                    rad= self.d[k]
                    deg= int(rad*180/PI)
                    symno= int(.5+ (rad+PI) / (PI/12) )
                    symno= symno % (len(asyms)-1)
                    strout= '%5.2f %3d (%s)' % (rad,deg,asyms[symno])
                elif k == 'skid': # A sensible interpretation of wheel spin.
                    frontwheelradpersec= self.d['wheelSpinVel'][0]
                    skid= 0
                    if frontwheelradpersec:
                        skid= .5555555555*self.d['speedX']/frontwheelradpersec - .66124
                    strout= bargraph(skid,-.05,.4,50,'*')
                elif k == 'slip': # A sensible interpretation of wheel spin.
                    frontwheelradpersec= self.d['wheelSpinVel'][0]
                    slip= 0
                    if frontwheelradpersec:
                        slip= ((self.d['wheelSpinVel'][2]+self.d['wheelSpinVel'][3]) -
                              (self.d['wheelSpinVel'][0]+self.d['wheelSpinVel'][1]))
                    strout= bargraph(slip,-5,150,50,'@')
                else:
                    strout= str(self.d[k])
            out+= "%s: %s\n" % (k,strout)
        return out

class DriverAction():
    '''What the driver is intending to do (i.e. send to the server).
    Composes something like this for the server:
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus 0)(meta 0) or
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus -90 -45 0 45 90)(meta 0)'''
    def __init__(self):
       self.actionstr= str()
       self.d= { 'accel':0.2,
                   'brake':0,
                  'clutch':0,
                    'gear':1,
                   'steer':0,
                   'focus':[-90,-45,0,45,90],
                    'meta':0
                    }

    def clip_to_limits(self):
        """There pretty much is never a reason to send the server
        something like (steer 9483.323). This comes up all the time
        and it's probably just more sensible to always clip it than to
        worry about when to. The "clip" command is still a snakeoil
        utility function, but it should be used only for non standard
        things or non obvious limits (limit the steering to the left,
        for example). For normal limits, simply don't worry about it."""
        self.d['steer']= clip(self.d['steer'], -1, 1)
        self.d['brake']= clip(self.d['brake'], 0, 1)
        self.d['accel']= clip(self.d['accel'], 0, 1)
        self.d['clutch']= clip(self.d['clutch'], 0, 1)
        if self.d['gear'] not in [-1, 0, 1, 2, 3, 4, 5, 6]:
            self.d['gear']= 0
        if self.d['meta'] not in [0,1]:
            self.d['meta']= 0
        if type(self.d['focus']) is not list or min(self.d['focus'])<-180 or max(self.d['focus'])>180:
            self.d['focus']= 0

    def __repr__(self):
        self.clip_to_limits()
        out= str()
        for k in self.d:
            out+= '('+k+' '
            v= self.d[k]
            if not type(v) is list:
                out+= '%.3f' % v
            else:
                out+= ' '.join([str(x) for x in v])
            out+= ')'
        return out
        return out+'\n'

    def fancyout(self):
        '''Specialty output for useful monitoring of bot's effectors.'''
        out= str()
        od= self.d.copy()
        od.pop('gear','') # Not interesting.
        od.pop('meta','') # Not interesting.
        od.pop('focus','') # Not interesting. Yet.
        for k in sorted(od):
            if k == 'clutch' or k == 'brake' or k == 'accel':
                strout=''
                strout= '%6.3f %s' % (od[k], bargraph(od[k],0,1,50,k[0].upper()))
            elif k == 'steer': # Reverse the graph to make sense.
                strout= '%6.3f %s' % (od[k], bargraph(od[k]*-1,-1,1,50,'S'))
            else:
                strout= str(od[k])
            out+= "%s: %s\n" % (k,strout)
        return out

def destringify(s):
    '''makes a string into a value or a list of strings into a list of
    values (if possible)'''
    if not s: return s
    if type(s) is str:
        try:
            return float(s)
        except ValueError:
            print("Could not find a value in %s" % s)
            return s
    elif type(s) is list:
        if len(s) < 2:
            return destringify(s[0])
        else:
            return [destringify(i) for i in s]

def drive_example(c):
    '''This is only an example. It will get around the track but the
    correct thing to do is write your own `drive()` function.'''
    S,R= c.S.d,c.R.d
    target_speed=160

    R['steer']= S['angle']*25 / PI
    R['steer']-= S['trackPos']*.25

    R['accel'] = max(0.0, min(1.0, R['accel']))
    

    if S['speedX'] < target_speed - (R['steer']*2.5):
        R['accel']+= .4
    else:
        R['accel']-= .2
    if S['speedX']<10:
       R['accel']+= 1/(S['speedX']+.1)

    if ((S['wheelSpinVel'][2]+S['wheelSpinVel'][3]) -
       (S['wheelSpinVel'][0]+S['wheelSpinVel'][1]) > 2):
       R['accel']-= 0.1



    R['gear']=1
    if S['speedX']>60:
        R['gear']=2
    if S['speedX']>100:
        R['gear']=3
    if S['speedX']>140:
        R['gear']=4
    if S['speedX']>190:
        R['gear']=5
    if S['speedX']>220:
        R['gear']=6
    return

# ================= MAIN DRIVE FUNCTION =================
def drive_modular(c):
    S, R = c.S.d, c.R.d
    R['steer'] = calculate_steering(S)
    R['accel'] = calculate_throttle(S, R)
    R['brake'] = apply_brakes(S)
    R['accel'] = traction_control(S, R['accel'])
    R['gear'] = shift_gears(S)
    return

# ================= MAIN LOOP =================
if __name__ == "__main__":
    #### DO NOT HARCODE THE PORT
    C = Client()
    for step in range(C.maxSteps, 0, -1):
        C.get_servers_input()
        drive_modular(C)
        C.respond_to_server()
    C.shutdown()
