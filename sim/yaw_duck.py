# The yaw duck (2026-11-04, user): the cube with its brain turned 90 deg
# forward, so its outputs face the sides, up and down; the brain ends up in
# front. The bottom output turns about a VERTICAL axis: one servo turns a
# bar that carries both legs. Each whole leg rides a short rail at the bar's
# end and slides along it with the bar's turn (slide_mm_deg: the planted one
# back, like the skis; in the real one a pin under the body in a slot of the
# leg's arm, yaw_mech_plot.py). The feet: a toe and a heel each, b out from
# the middle; optionally on a pivot tied to the bar (k).
#
# Why it rocks on: standing on one foot, the bar is held by the floor, so
# the servo turns the BODY about its vertical axis. The body's centre of
# mass sits y_b ahead of that axis (the brain is in front), so turning it
# swings the centre of mass sideways, and sideways is the rocking:
#     I0 theta'' = -M g R sin(alpha - |theta|) sgn(theta) + m_b h y_b psi''
# (psi: the body's turn on the bar; ski.py's yaw term with the pivot on the
# servo's axis). The rocking sets the rhythm, as in ski.py: on the right
# foot the servo goes to +dir*turn_deg, on the left to -dir*turn_deg.
#
# And why it goes: the planted leg sliding back carries the body forward.
#
#   python yaw_duck.py                     the default walk, printed
#   python yaw_duck.py --view --set k=0.25

import argparse
import math

import mujoco
import numpy as np

YAW = dict(
    cube_g=125.0,      # g — the cube
    brain_g=60.0,      # g — the brain, now in front of the cube
    brain_y=42.0,
    brain_up=0.0,      # 1: the brain on top of the cube instead (no yaw push then)      # mm — its centre ahead of the cube's
    brain_cube=0.0,    # 1: the brain a whole 56 mm cube, its centre brain_y ahead
    brain_z=0.0,       # mm — and this high (56: on top of the motor cube)
    head_z=-999.0,     # mm — the neck's height on the cube (-999: on top)
    yaw_hold=0.0,      # deg of feet turn per deg of heading error (a gyro; needs
    yaw_hold_i=0.0,    #    foot_yaw on), plus this per deg·s; from yaw_hold_s on,
    yaw_hold_s=3.0,    #    holding the heading it had then; at most yaw_hold_max
    yaw_hold_max=8.0,
    yaw_sign=1.0,      # which way the feet's turn turns the body
    gyro_bias=0.0,     # deg/s — the gyro's own drift
    gyro_rock=0.0,     # 1: the legs' clock locked to the rocking read by an IMU
    gyro_s=3.0,        #    (2: read only, logged in p["_gy"]) from here on;
    gyro_off=0.0,      #    cycles — the clock's place against the rocking
    gyro_kp=4.0,       #    the lock's pull (1/s) and the rhythm's (Hz/s per cycle)
    gyro_ki=1.0,
    gyro_amp_on=1.0,   #    deg — no lock under this rocking (phase meaningless)
    gyro_amp_k=0.0,    # head deg per deg of rocking short of gyro_amp_target
    gyro_amp_target=4.0,
    gyro_amp_tau=0.3,  # s — the size's smoothing (each landing jolts the reading)
    gyro_stop_s=0.0,   # s — >0: the target eases to 0 here (the head brakes)
    gyro_stop_ease=0.5,
    halves_y=0.0,      # mm — >0: two half cubes (28 mm thick) this far ahead and
    half_front_g=0.0,  #    behind, with these masses (battery, CPU); set brain_g 0
    half_back_g=0.0,
    hip=70.0,          # mm — the cube's centre above the floor
    bar_g=8.0,         # g — the bar on the bottom output, and its posts
    foot_g=20.0,       # g — each foot (its toe and heel, half each)
    b=23.5,            # mm — the feet out from the middle
    foot_len=100.0,    # mm — toe to heel
    par_L=0.0,         # mm — >0: the leg a parallelogram this long: the foot,
    par_s0=0.0,        #    still level, rises as it moves from s0 (its lowest)
    toe_flat=0.0,      # mm — >0 (arc soles): the sole flat only to here ahead of
    toe_r=40.0,        #    the foot's middle, then curving up with this radius
    foot_y=0.0,        # mm — the feet's middle ahead of the servo's axis
    ball_r=3.0,        # mm — toe and heel balls
    ankle_k=0.0,       # N m/rad — >0: each foot on an ankle, toe up and down
                       #    against this spring (user, 04/11)
    ankle_c=0.005,     # N m s/rad — its damping
    ankle_h=8.0,       # mm — the ankle's hinge above the balls
    ankle_range=20.0,  # deg — how far the toe can go each way
    ankle_roll_k=0.0,  # N m/rad — >0: a side-to-side ankle with this spring
    ankle_roll_c=0.002,  # N m s/rad — its damping
    ankle_roll_range=10.0,  # deg — then the foot's edge lifts
    foot_w=0.0,        # mm — >0: each foot this wide (4 balls), for the
                       #    side-to-side ankle
    leg_k=0.0,         # N/m — >0: each foot on a vertical spring in its leg
    leg_c=1.0,         # N s/m — the spring's damping
    leg_travel=15.0,   # mm — how far the foot can go up into the leg
    leg_pre=0.0,       # mm — preload: the spring set this far below its stop,
                       #    so the foot holds rigid until the load passes
                       #    k * leg_pre (then only the landings squeeze it)
    sole_r=0.0,        # mm — >0: instead of toe and heel balls, each sole is an
                       #    arc side to side, this radius, centred on the
                       #    middle (the first duck's): the rhythm is set by the
                       #    arc, not by how far it rocks (user, 04/11: 1 Hz)
    sole_in=3.5,       # mm — the arc soles from here out to sole_out (from the
    sole_out=60.0,     #    middle), each foot_len long
    sole_flat=0.0,     # mm — >0: the arc only to here, then straight along
                       #    its tangent out to sole_out
    k=0.0,             # each foot's pivot tied to the bar: it turns -k times
                       #    the bar (user, 04/11: one servo for the yaw and the
                       #    straight travel). 0: feet fixed to the bar
                       #    (yaw only); 1: always parallel to the body
    slide_amp=0.0,     # mm — >0: the legs slide on their own servo (not tied
                       #    to the turn), +-this, the planted one back
    slide_speed=150.0, # mm/s — its speed
    slide_dead=0.3,    # deg — it switches when the tilt passes this
    slide_phase=0.0,   # cycles — open loop: the legs this late on the clock
    yaw_off_s=0.0,     # s — >0: the yaw servo stops (eased over 1 s), or with
    yaw_brake_s=0.0,   #    yaw_brake_s turns against the rocking that long first;
    yaw_on_s=0.0,      #    from yaw_on_s it swings again (eased in, 1 s)
    spin_mm=0.0,       # mm — >0: from turn_s, turn on the spot: the legs go
    spin_win=0.1,      #    opposite ways in the double supports (windows this
    spin_c=0.98,       #    many cycles wide, centred at spin_c and +0.5) and
                       #    come back in the air (spin_profile); - turns the other way
    hold_r=0.0,        # mm — from turn_s, the right leg held this far forward
    hold_l=0.0,        #    and the left (helping the yaw ratchet)
    kick_mm=0.0,       # mm — and the right leg kicked forward this far, in kick_win
    kick_win=0.06,     #    cycles from kick_at, then back slowly before it lands
    kick_at=0.5,
    osc_r=0.0,         # mm — and the right leg swinging fore and aft this much,
    osc_ph=0.0,        #    most forward at this cycle
    fb_n=0.0,          # >0: from turn_s, fb_n cycles stepping forward, fb_n back
    yaw_flip=0.0,      # 1: from turn_s the yaw swings the other way round
    foot_yaw=0.0,      # deg — >0 (with slide_amp): a servo turns each foot this
                       #    much in the air and back while planted, from turn_s
    foot_shared=0.0,
    swing_k=1.0,       # leg_profile's k: 1 lands moving with the planted speed, 0 stops
    swing_k_s=0.0,     # s — >0: k eases (1 s) to swing_k2 from here
    swing_k2=1.0,
    contact_phase=0.0, # 1: each landing (a switch under the foot) resets the legs'
    contact_off=0.0,   #    phase (right lands at contact_off, left half a cycle on)
    contact_gain=0.3,  #    each landing moves the phase this share of its error,
    contact_fgain=0.2, #    and the rhythm by this share (a soft phase lock)
    feet_pump=0.0,     # 1 (with foot_yaw servos): the feet do the yaw, the bar's servo idle   # 1: one servo for both feet, mirrored; 2: both the same way
    turn_e=0.0,        # s — >0: the feet's turn (foot_yaw) back to straight from here
    ankle_servo=0.0,   # 1: a servo rolls each foot under its leg (user, 07/11:
    ankle_amp=0.0,     #    the rocking driven by the ankles) — deg, both feet the
    ankle_ph=0.0,      #    same way, a sine; its phase on the clock (cycles)
    ankle_hz=0.0,      #    and its rhythm (0: the clock's)
    ankle_s=0.0,       # s — the ankles start (eased in over 1 s)
    ank_b_s=0.0,       # s — then their size eases to ank_b (deg; < 0 damps)
    ank_b=0.0,
    ank_c_s=0.0,
    ank_c=0.0,
    legs_stop_s=0.0,   # s — the legs' motor stops: the legs stay where they are
    ankle_fb=0.0,      # 1: the ankles in closed loop on the rocking (an IMU):
    ankle_fb_s=0.0,    #    from here, rolled with the rocking's speed to bring
    ankle_target=0.0,  #    its size to this (deg; 0 = stop it),
    ankle_g=1.0,       #    gain (ankle rad per rad of size error),
    ankle_max=6.0,     #    at most this (deg)
    ankle_f0=0.0,      #    the rocking's own rhythm for the estimate (0: the clock)
    ankle_limit=0.0,   # 1: the loop only brakes (when the rocking passes the target)
    ank_t_b_s=0.0,     # s — the target changed on the way (eased over 1 s)
    ank_t_b=0.0,
    ank_t_c_s=0.0,
    ank_t_c=0.0,
    jansen=0.0,        # 1 (with clock_hz, slide_amp): each foot follows Theo Jansen's
                       #    leg (user, 09/11): one crank per leg (or one motor,
                       #    half a turn apart), the stride 2 slide_amp, the lift
                       #    scaled with it; it starts and stops with the feet
                       #    changing over (both down)
    jansen_lift=1.0,   #    the lift times this (1: the linkage's own)
    lift_dir_mm=0.0,   # mm — >0: each foot rises this much while its leg goes
    lift_dir_v=5.0,    #    forward faster than this (mm/s), down otherwise
    lift_tau=0.03,     #    (a one-way mechanism, user 09/11), taking this long (s)
    head=0.0,          # 1: a head on a neck servo on top of the cube, tilting side
    head_g=47.0,       #    to side (user, 09/11: it starts, keeps and stops the
    head_h=50.0,       #    rocking) — g, its centre this far above the neck (mm),
    head_take=1.0,     #    1: its mass taken from the cube (same total)
    ballast_g=0.0,     # g — fixed weights on the cube (taken from it, same total):
    ballast_y=0.0,     #    one this far ahead (mm) and, with ballast_pair 1, the
    ballast_pair=0.0,  #    same behind (only yaw inertia, centre of mass kept)
    head_y=0.0,        # mm — the neck this far ahead of the cube's centre (user, 09/11)
    head_speed=10.5,   # rad/s — SG90-like, 0.1 s / 60 deg
    head_amp=0.0,      # deg — a sine on the legs' clock (or square, head_wave 1),
    head_ph=0.0,       #    this many cycles late
    head_wave=0.0,
    head_s=0.0,        # s — the head starts here
    head_kick=0.0,     # deg — first a square swing this big (the kick out of
    head_kick_s=0.0,   #    standing on both inner edges) for this long
    head_b_s=0.0,      # s — its size eased (1 s) to head_b from here
    head_b=0.0,
    head_flip_s=0.0,   # s — from here half a cycle out of step (brakes)
    head_off_s=0.0,    # s — from here back to straight (and held),
    head_off_speed=0.0,  #  at this speed (rad/s; 0: head_speed)
    head_ph_b_s=0.0,   # s — from here the head's phase slides by head_ph_b cycles
    head_ph_b=0.0,     #    (0.5: from pumping to braking, smoothly, no jump)
    head_ph_ease=0.5,  #    over this long
    head_land_s=0.0,   # s — >0 (with contact_phase): the brake slide starts at the
    head_land_dly=0.0, #    first right landing after this, plus this delay (s)
    head_fb_land=0.0,  # +1/-1 (with contact_phase, from head_land_s): at each
    head_fb_amp=6.0,   #    landing a half-sine of the head this big (deg) to one
    head_fb_half=0.0,  #    side (sign), lasting this long (0: half a cycle)
    head_ph_c_s=0.0,   # s — and another slide (e.g. standing 0.375 -> walking 0.5)
    head_ph_c=0.0,
    head_ph_c_ease=1.0,
    ease_s=1.0,        # s — how long the step (amp_b, amp_c) and the head (head_b) take to change
    head_brake_s=0.0,  # s — from the next pass through the middle, one half
                       #    cycle the other way (a smooth brake), then still
    foot_kp=0.0,       # N·m/rad — >0: a torsion spring between each foot's servo and
    foot_damp=0.0,     #    the foot (user, 07/11: passive yaw), and its damping N·m·s/rad
    foot_yaw_b_s=0.0,  # s — >0: from here the feet's turn eases to foot_yaw_b (deg),
    foot_yaw_b=0.0,
    foot_yaw_c_s=0.0,  #    and from here to foot_yaw_c (a route: left, straight, right)
    foot_yaw_c=0.0,
    turn_s=0.0,        # s — >0: turning from here (eased in over 1 s) by
    step_diff=0.0,     #    mm — the right leg's step this much longer than the left
    yaw_bias=0.0,      #    deg — and/or the yaw servo off centre
    yaw_skew=0.0,
    yaw_fast=0.0,      #    rad/s — and/or the servo this fast one way (the
    yaw_fast_sign=1.0, #    sign of that way), servo_speed the other      #    and/or its turn on the right foot (1 + skew) times bigger
    clock_b_s=0.0,     # s — >0: the clock eases to clock_b Hz from here,
    clock_b=0.0,       #    and to clock_c from clock_c_s
    clock_c_s=0.0,
    clock_c=0.0,
    amp_b_s=0.0,       # s — >0: from here the step eases (1 s) to amp_b mm,
    amp_b=0.0,         #    and from amp_c_s to amp_c: speed changes and stops
    amp_c_s=0.0,
    amp_c=0.0,
    slide_per_leg=0.0,  # 1: open loop, a servo per leg: steady push planted,
                       #    swing forward raised, landing already going back
    slide_sine=0.0,    # 1: open loop, the legs swing smoothly (a sine of the
                       #    clock) instead of ramps with sudden reversals
    legs_start_s=0.0,  # s — open loop: the legs start this late (the yaw
    legs_ramp_s=1.0,   # s — the legs' step grows from 0 to full over this long
                       #    first, user 04/11)
    slide_kp=2000.0,   # N/m — its position loop
    slide_crank=15.0,  # mm — its crank: the servo's torque over this
    slide_mm_deg=0.33, # mm per deg of the bar: each whole leg also slides
                       #    straight on a rail at the bar's end, the planted
                       #    one back (user, 04/11)
    friction=0.8,
    impratio=1.0,      # MuJoCo: >1 makes friction stiffer (less creep under a held force)
    cone_elliptic=0.0, # 1: elliptic friction cone (slide and spin share it exactly)
    turn_deg=15.0,     # deg — the servo's turn each way (04/11: 15 with 0.33 mm/deg walks ~15 cm/s)
    dir=-1.0,          # +1 / -1: which way it turns on which foot (-1 feeds the
                       #    rocking with a damped servo, 2026-11-04; +1 kills it)
    dead_deg=0.3,      # deg — near upright: hold
    start_s=0.0,       # s — >0: from standing, the yaw servo swings on its own
    start_hz=1.3,      #    at this rhythm, +-start_deg, for up to start_s or
    start_deg=10.0,    #    until the tilt passes start_until_deg; then the
    start_until_deg=3.0,  #    tilt runs it as usual
    clock_hz=0.0,      # Hz — >0: open loop, the servo switches on a fixed clock
                       #    (no tilt sensor); 0: the rocking sets it
    clock_phase=0.0,   # its phase, in cycles, at the release
    clock_hz0=0.0,     # Hz — >0: the clock starts here and eases to clock_hz
    clock_ramp_s=0.0,  #    over this many seconds (a fast start, then slower)
    servo_speed=10.0,  # rad/s — MG90D, 0.1 s / 60 deg
    servo_torque=0.2,  # N m
    servo_kp=3.0,      # N m/rad — its stiffness (the position loop)
    servo_damp=0.02,   # N m s/rad — its damping: 0.02 settles the turn without
                       #    ringing (0.002 rang at ~20 Hz, +-22 deg for +-10)
    secs=12.0,
    slow=2.0,          # the viewer: this many times slower than real time
    show_forces=0.0,   # 1: the viewer draws the floor's push on each ball
)


def sole_profile(p):
    """The curved sole seen from the front (mm): x outward from the middle,
    from sole_in to sole_out, z up from the lowest point. An arc of radius
    sole_r; past sole_flat (if set) it carries on straight along the arc's
    tangent there (user, 04/11: curve, then flat). The same points make the
    simulator's mesh and the equations' profile (yaw_profile_model.py)."""
    rho = p["sole_r"]
    xt = p["sole_flat"] or p["sole_out"]
    xs = np.linspace(p["sole_in"], p["sole_out"], 33)
    if p["sole_flat"]:
        xs = np.union1d(xs, [xt])
    zt = rho - math.sqrt(rho * rho - xt * xt)
    slope = xt / math.sqrt(rho * rho - xt * xt)
    zs = [rho - math.sqrt(rho * rho - x * x) if x <= xt else zt + slope * (x - xt) for x in xs]
    return list(xs), zs


def spin_profile(ph, D, w, c):
    """The right leg turning on the spot (user, 04/11: turn on the way out,
    not on the way back): it goes forward D/2 in each double support (both
    feet down, windows w wide centred at cycle c and c + 0.5, while the left
    goes back as much, so the bar must turn), holds still while planted
    alone, and goes back D while in the air. The left: -spin(ph + 0.5)."""
    def S(x):
        x = min(1.0, max(0.0, x))
        return 3 * x * x - 2 * x ** 3
    x = (ph - (c + 0.5 + w / 2)) % 1.0     # from the end of the right's lift-off window
    air = 0.5 - w - 0.04                   # the return, inside the air phase
    if x < air + 0.02:
        return D / 2 - D * S((x - 0.02) / air)
    if x < 0.5 - w:
        return -D / 2
    if x < 0.5:
        return -D / 2 + D / 2 * S((x - (0.5 - w)) / w)
    if x < 1.0 - w:
        return 0.0
    return D / 2 * S((x - (1.0 - w)) / w)


def leg_profile2(ph, A, A_next):
    """leg_profile with the next cycle's step different (A_next): the raised
    half goes from -A to A_next, leaving at A's speed, landing at A_next's."""
    if ph < 0.5:
        return A - 4 * A * ph
    u = (ph - 0.5) / 0.5
    h00, h10 = 2*u**3 - 3*u**2 + 1, u**3 - 2*u**2 + u
    h01, h11 = -2*u**3 + 3*u**2, u**3 - u**2
    return h00*(-A) + h10*(-2*A) + h01*A_next + h11*(-2*A_next)


def jansen_path(n=720):
    """Theo Jansen's leg (his 'holy numbers', crank 15): the foot's place
    over one turn of the crank (user, 09/11). Returns (back, up) per sample,
    in the numbers' units, the turn starting where the foot lands (the
    lowest stretch begins), 'back' growing while planted."""
    a, b, c, d, e, f, g, h, i, j, k, l, m = 38, 41.5, 39.3, 40.1, 55.8, 39.4, 36.7, 65.7, 49, 50, 61.9, 7.8, 15
    B = np.array([-a, -l])
    def cc(P1, r1, P2, r2, side):
        dv = P2 - P1
        L = math.hypot(*dv)
        x = (r1 ** 2 - r2 ** 2 + L ** 2) / (2 * L)
        u = dv / L
        return P1 + x * u + side * math.sqrt(max(r1 ** 2 - x ** 2, 0.0)) * np.array([-u[1], u[0]])
    P = []
    for t in np.linspace(0, 2 * math.pi, n + 1)[:-1]:
        C = m * np.array([math.cos(t), math.sin(t)])
        D, E = cc(C, j, B, b, -1), cc(C, k, B, c, 1)
        F = cc(B, d, D, e, 1)
        G = cc(F, f, E, g, -1)
        P.append(cc(G, h, E, i, -1))
    P = np.array(P)
    on = P[:, 1] < P[:, 1].min() + 1.0          # the flat stretch
    i0 = next(q for q in range(n) if on[q] and not on[q - 1])   # where it lands
    P = np.roll(P, -i0, axis=0)
    back = P[:, 0] - 0.5 * (P[:, 0].max() + P[:, 0].min())
    return back, P[:, 1] - P[:, 1].min()


def leg_profile(ph, A, k=1.0):
    """One leg's place over a cycle (0..1): planted the first half, from +A
    back to -A at a steady speed; raised the second, a cubic from -A to +A
    that leaves and lands with that same backward speed (times k: user,
    05/11, 0 stops at both ends, a gentler swing)."""
    if ph < 0.5:
        return A - 4 * A * ph
    u = (ph - 0.5) / 0.5
    m = -2 * A * k                     # the steady speed over the half, x duration
    h00, h10 = 2 * u ** 3 - 3 * u ** 2 + 1, u ** 3 - 2 * u ** 2 + u
    h01, h11 = -2 * u ** 3 + 3 * u ** 2, u ** 3 - u ** 2
    return h00 * (-A) + h10 * m + h01 * A + h11 * m


def spring_open(p):
    """leg_k > 0: the foot hangs from the leg on a vertical spring (user,
    04/11), free to go up into the leg by up to leg_travel; the weight
    squeezes it M g / (2 k) standing on both feet. The body to close after
    the foot's geoms."""
    if not p["leg_k"]:
        return ""
    return ('<geom type="sphere" size=".002" mass="0.0005" contype="0" conaffinity="0"/><body>'
            f'<joint type="slide" axis="0 0 1" stiffness="{p["leg_k"]}" damping="{p["leg_c"]}"'
            f' springref="{-p["leg_pre"] / 1000}"'
            f' limited="true" range="-0.0005 {p["leg_travel"] / 1000}" armature="0.00001"/>')


def ankle_open(p, zf):
    """ankle_k > 0: the foot hangs from the leg on an ankle (user, 04/11):
    a hinge across (X) ankle_h above the balls, so the toe can go up or down
    against a spring, the foot staying flat while the leg pitches. Its body
    to close after the foot's geoms."""
    if not p["ankle_k"]:
        return ""
    za = zf + p["ankle_h"] / 1000
    lim = math.radians(p["ankle_range"])
    return ('<geom type="sphere" size=".002" mass="0.0005" contype="0" conaffinity="0"/>'
            '<body><joint type="hinge" axis="1 0 0"'
            f' pos="0 0 {za}" stiffness="{p["ankle_k"]}" damping="{p["ankle_c"]}"'
            f' limited="true" range="{-lim} {lim}" armature="0.000001"/>')


def roll_ankle_open(p, zf, side=""):
    """ankle_roll_k > 0: a side-to-side ankle (user, 04/11): a hinge along
    (Y) ankle_h above the balls, so the leg can lean over a flat foot
    against a spring, storing the rocking instead of losing it in a landing.
    Up to ankle_roll_range each way, then the foot's edge lifts. Wants a
    foot with width (foot_w). Its body to close after the foot's geoms."""
    if not (p["ankle_roll_k"] or p["ankle_servo"]):
        return ""
    za = zf + p["ankle_h"] / 1000
    lim = math.radians(p["ankle_roll_range"])
    return ('<geom type="sphere" size=".002" mass="0.0005" contype="0" conaffinity="0"/>'
            f'<body><joint name="aroll{side}" type="hinge" axis="0 1 0"'
            f' pos="0 0 {za}" stiffness="{p["ankle_roll_k"]}" damping="{p["ankle_roll_c"]}"'
            f' limited="true" range="{-lim} {lim}" armature="0.000001"/>')


def build(p):
    mm, kg = (lambda v: v / 1000.0), (lambda g: g / 1000.0)
    b, L2, r = mm(p["b"]), mm(p["foot_len"] / 2), mm(p["ball_r"])
    yf = mm(p["foot_y"])
    zb = -0.028                                    # the bottom output
    zf = r - mm(p["hip"]) - zb                     # balls' centres, in the bar frame
    feet, ties, meshes = "", "", ""
    for side, s in (("R", 1), ("L", -1)):
        # each foot on a vertical pivot at the bar's end, tied to the bar's
        # turn: foot = -k * bar (k 0: fixed to the bar; 1: always parallel
        # to the body, the bar's turn all straight travel)
        balls = "".join(f'<geom name="{side}{k}" type="sphere" pos="0 {y} {zf}" size="{r}"'
                        f' mass="{kg(p["foot_g"] / 2)}" rgba=".95 .6 .2 1"/>'
                        for k, y in (("t", yf + L2), ("h", yf - L2)))
        if p["foot_w"]:                # a foot with width: toe and heel at
            w2 = mm(p["foot_w"] / 2)   # both its edges (outer named as before)
            balls = "".join(f'<geom name="{side}{k}{n}" type="sphere" pos="{s * dx} {y} {zf}" size="{r}"'
                            f' mass="{kg(p["foot_g"] / 4)}" rgba=".95 .6 .2 1"/>'
                            for k, y in (("t", yf + L2), ("h", yf - L2))
                            for n, dx in (("", w2), ("i", -w2)))
        if p["sole_r"]:                # an arc slab: convex, so a mesh will do
            xa, za = sole_profile(p)    # mm, outward from the middle, up
            xs = s * np.asarray(xa) / 1000
            zs = [z / 1000 - mm(p["hip"]) - zb for z in za]
            # fore and aft: flat, or flat to toe_flat ahead of the foot's
            # middle and then curving up (radius toe_r) to the toe (user,
            # 05/11: past it the foot tips onto one rounded point to turn)
            ys, dzs = [yf - L2, yf + L2], [0.0, 0.0]
            if p["toe_flat"]:
                y0, rt = yf + mm(p["toe_flat"]), mm(p["toe_r"])
                ys = [yf - L2] + list(np.linspace(y0, yf + L2, 9))
                dzs = [0.0] + [rt - math.sqrt(rt * rt - (y - y0) ** 2) for y in ys[1:]]
            ztop = max(zs) + max(dzs) + 0.004
            v = []
            for x, z in zip(xs, zs):
                for y, dz in zip(ys, dzs):
                    v += [x - s * b, y, z + dz, x - s * b, y, ztop]
            meshes += f'<mesh name="sole{side}" vertex="{" ".join(f"{q:.6f}" for q in v)}"/>'
            balls = (f'<geom name="{side}s" type="mesh" mesh="sole{side}" mass="{kg(p["foot_g"])}"'
                     ' rgba=".95 .6 .2 1"/>')
        # the whole leg rides a carriage on a short rail at the bar's end,
        # fore and aft (user, 04/11: the leg slides, not the sole); the foot
        # under it on the pivot
        feet += (f'<body name="leg{side}" pos="{s * b} 0 0">'
                 f'<joint name="slide{side}" type="slide" axis="0 1 0" damping="0.05" armature="0.0001"/>'
                 + (f'<joint name="lift{side}" type="slide" axis="0 0 1" damping="0.05" armature="0.0001"/>'
                    if p["par_L"] or p["jansen"] or p["lift_dir_mm"] else "")
                 + f'<geom type="box" size=".005 .008 .003" mass="0.00001" contype="0" conaffinity="0"'
                 ' rgba=".4 .4 .45 1"/>'
                 f'<geom type="capsule" fromto="0 0 0 0 0 {zf}" size=".003"'
                 ' mass="0.00001" contype="0" conaffinity="0" rgba=".95 .6 .2 1"/>'
                 f'<body name="foot{side}">'
                 f'<joint name="pivot{side}" type="hinge" axis="0 0 1" damping="{p["foot_damp"] or 0.0005}" armature="0.00001"/>'
                 + spring_open(p) + ankle_open(p, zf) + roll_ankle_open(p, zf, side) + balls +
                 f'<geom type="capsule" fromto="0 {yf - L2} {zf} 0 {yf + L2} {zf}" size=".0015"'
                 ' mass="0.00001" contype="0" conaffinity="0" rgba=".95 .6 .2 .5"/>'
                 + ("</body>" if p["ankle_k"] else "")
                 + ("</body>" if p["ankle_roll_k"] or p["ankle_servo"] else "")
                 + ("</body>" if p["leg_k"] else "") + '</body></body>')
        if p["par_L"]:                 # a parallelogram leg (user, 05/11): the
            L_, s0 = mm(p["par_L"]), mm(p["par_s0"])   # foot on an arc of the
            a1, a2 = -s0 / L_, 1 / (2 * L_)            # leg's length, lowest at
            ties += (f'<joint joint1="lift{side}" joint2="slide{side}"'   # s0
                     f' polycoef="0 {a1} {a2} 0 0" solref="0.002 1"/>')
        if not p["foot_yaw"]:          # else each foot turned by its own servo
            ties += (f'<joint joint1="pivot{side}" joint2="turn" polycoef="0 {-p["k"]} 0 0 0"'
                     ' solref="0.002 1"/>')
        # the leg slides tied to the bar too: on the right +g per rad of
        # the bar, on the left -g, so the planted one goes back while its
        # turn runs (scissors, like the skis)
        g_m = p["slide_mm_deg"] * 180 / math.pi / 1000
        if not p["slide_amp"]:         # the slide tied to the turn (one servo)
            ties += (f'<joint joint1="slide{side}" joint2="turn" polycoef="0 {s * g_m} 0 0 0"'
                     ' solref="0.002 1"/>')
    # a second servo for the slides (user, 04/11): a crank of slide_crank
    # on another output pushes both carriages, opposite ways; position
    # loops, the force its torque over the crank
    fmax = p["servo_torque"] / (p["slide_crank"] / 1000)
    slide_acts = "".join(f'<position name="s{side}" joint="slide{side}" kp="{p["slide_kp"]}"'
                         f' forcerange="-{fmax} {fmax}"/>' for side in "RL") if p["slide_amp"] else ""
    if p["foot_yaw"]:                  # a servo turning each foot (user, 05/11)
        slide_acts += "".join(f'<position name="p{side}" joint="pivot{side}" kp="{p["foot_kp"] or p["servo_kp"]}"'
                              f' forcerange="-{p["servo_torque"]} {p["servo_torque"]}"/>' for side in "RL")
    if p["ankle_servo"]:               # a servo rolling each foot under its leg
        slide_acts += "".join(f'<position name="a{side}" joint="aroll{side}" kp="{p["servo_kp"]}"'
                              f' forcerange="-{p["servo_torque"]} {p["servo_torque"]}"/>' for side in "RL")
    if p["brain_cube"]:                # the brain a whole cube in line ahead (user,
        brain = (f'<geom type="box" size=".028 .028 .028" pos="0 {mm(p["brain_y"])} {mm(p["brain_z"])}" mass="{kg(p["brain_g"])}"'
                 ' rgba=".6 .7 .85 1" contype="0"/>')   # 10/11): ports on 5 walls
    elif p["brain_up"]:                # back on top (user, 05/11): the duck upright
        brain = (f'<geom type="box" size=".028 .028 .014" pos="0 0 .042" mass="{kg(p["brain_g"])}"'
                 ' rgba=".6 .7 .85 1" contype="0"/>')
    else:
        brain = (f'<geom type="box" size=".028 .014 .028" pos="0 {mm(p["brain_y"])} 0" mass="{kg(p["brain_g"])}"'
                 ' rgba=".6 .7 .85 1" contype="0"/>')
    head, cube_g = "", p["cube_g"]
    if p["halves_y"]:                  # the brain split (user, 10/11): battery half
        for hy, hg, rgba in ((p["halves_y"], p["half_front_g"], ".85 .75 .4 1"),   # ahead,
                             (-p["halves_y"], p["half_back_g"], ".6 .7 .85 1")):  # CPU half behind
            head += (f'<geom type="box" size=".028 .014 .028" pos="0 {mm(hy)} 0" mass="{kg(hg)}"'
                     f' rgba="{rgba}" contype="0"/>')
    if p["ballast_g"]:                 # fixed weights (user, 09/11: why the head
        nb = 2 if p["ballast_pair"] else 1   # ahead keeps it straight)
        cube_g -= p["ballast_g"]
        for sy in ((1, -1) if nb == 2 else (1,)):
            head += (f'<geom type="sphere" size=".008" pos="0 {sy * mm(p["ballast_y"])} 0"'
                     f' mass="{kg(p["ballast_g"] / nb)}" rgba=".3 .3 .3 1" contype="0"/>')
    if p["head"]:                      # a neck servo on top of the cube (axis fore
        cube_g -= p["head_g"] if p["head_take"] else 0.0   # and aft), the head on it
        hh = mm(p["head_h"])
        zn = .056 if p["brain_up"] and not p["brain_cube"] else .028   # on top of the brain if it is up
        if p["head_z"] > -900: zn = mm(p["head_z"])   # e.g. 0: the front wall's output
        head += (f'<body name="head" pos="0 {mm(p["head_y"])} {zn}">'
                '<joint name="neck" type="hinge" axis="0 1 0" damping="0.002" armature="0.00001"/>'
                f'<geom type="capsule" fromto="0 0 0 0 0 {hh}" size=".003" mass="0.00001" contype="0" conaffinity="0"/>'
                f'<geom type="box" size=".018 .018 .018" pos="0 0 {hh}" mass="{kg(p["head_g"])}"'
                ' rgba=".55 .5 .85 1" contype="0"/></body>')
        slide_acts += (f'<position name="neck" joint="neck" kp="{p["servo_kp"]}"'
                       f' forcerange="-{p["servo_torque"]} {p["servo_torque"]}"/>')
    if p["jansen"] or p["lift_dir_mm"]:   # the foot's height too (Jansen's leg: the
        slide_acts += "".join(f'<position name="z{side}" joint="lift{side}" kp="{p["slide_kp"]}"'   # linkage
                              f' forcerange="-{fmax} {fmax}"/>' for side in "RL")   # sets it)
    xml = f"""
<mujoco model="yaw_duck">
  <option timestep="0.0002" gravity="0 0 -9.81" impratio="{p['impratio']}" cone="{'elliptic' if p['cone_elliptic'] else 'pyramidal'}"/>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="256" height="256"
             rgb1=".85 .85 .85" rgb2=".75 .75 .75"/>
    <material name="grid" texture="grid" texrepeat="20 20"/>
    {meshes}
  </asset>
  <default>
    <geom contype="1" conaffinity="0" friction="{p['friction']} 0.005 0.0001" solref="0.002 1"/>
  </default>
  <worldbody>
    <light pos="0 -0.3 0.6" dir="0 0.5 -1"/>
    <geom name="floor" type="plane" size="2 2 0.01" material="grid" contype="0" conaffinity="1"/>
    <body name="torso" pos="0 0 {mm(p['hip'])}">
      <freejoint/>
      <geom type="box" size=".028 .028 .028" mass="{kg(cube_g)}" rgba=".75 .8 .9 1" contype="0"/>
      {brain}
      {head}
      <geom type="box" size=".006 .003 .006" pos="0 {mm(p['brain_y']) + .016} .02" mass="0.0001"
            rgba=".9 .2 .2 1" contype="0"/>
      <body name="bar" pos="0 0 {zb}">
        <joint name="turn" type="hinge" axis="0 0 1" damping="{p['servo_damp']}" armature="0.0001"/>
        <geom type="capsule" fromto="{-b} 0 0 {b} 0 0" size=".003" mass="{kg(p['bar_g'])}"
              rgba=".95 .6 .2 1" contype="0" conaffinity="0"/>
        {feet}
      </body>
    </body>
  </worldbody>
  <equality>{ties}</equality>
  <actuator>
    <position name="turn" joint="turn" kp="{p['servo_kp']}" forcerange="-{p['servo_torque']} {p['servo_torque']}"/>
    {slide_acts}
  </actuator>
</mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    return m, mujoco.MjData(m)


def release(p, theta0=math.radians(8), view=False, hook=None):
    """Tilted theta0 onto the right foot's outer line (x = b), let go.
    Rows: t, roll, body heading, bar heading, x, y (mm... m), servo angle."""
    m, d = build(p)
    b, z0 = p["b"] / 1000, p["hip"] / 1000
    c, s = math.cos(theta0), math.sin(theta0)
    rho = p["sole_r"] / 1000
    def place():
        mujoco.mj_resetData(m, d)
        if rho:                        # rolled theta0 on the arc: about its centre
            d.qpos[0] = (z0 - rho) * s
            d.qpos[2] = (z0 - rho) * c + rho + 0.0002
        else:                          # tipped onto the right foot's outer line
            d.qpos[0] = b - b * c + z0 * s
            d.qpos[2] = b * s + z0 * c + 0.0002
        d.qpos[3:7] = [math.cos(theta0 / 2), 0, math.sin(theta0 / 2), 0]
        mujoco.mj_forward(m, d)
    place()
    bar = m.body("bar").id
    T, dead = math.radians(p["turn_deg"]), math.radians(p["dead_deg"])
    dv = p["servo_speed"] * m.opt.timestep
    cmd = [0.0]
    scmd = [0.0]                       # the slide servo: right leg's place
    clock_cyc = [0.0]                  # the open-loop clock's cycles
    clock_int = [0.0]
    held = {}                          # what stays put once stopped
    ank_cyc = [0.0]                    # the ankles' own clock (ankle_hz)
    imu = {"th": 0.0, "w": 0.0}       # the closed loop's reading
    gy = dict(th=0.0, w=0.0, amp=0.0, ph=0.0, f=p["clock_hz"] or 1.5, log=[])
    p["_gy"] = gy                      # the gyro's rocking reading (gyro_rock)
    jqa = [m.joint("arollR").qposadr[0], m.joint("arollL").qposadr[0]] if p["ankle_servo"] else []
    ida = [m.actuator("aR").id, m.actuator("aL").id] if p["ankle_servo"] else []
    dvs = p["slide_speed"] / 1000 * m.opt.timestep
    hcmd = [0.0]                       # the neck servo's command
    idh = m.actuator("neck").id if p["head"] else -1
    if p["jansen"]:                    # the foot's path, scaled to the stride
        jb, ju = jansen_path()
        js = 2 * p["slide_amp"] / 1000 / np.ptp(jb)
    if p["jansen"] or p["lift_dir_mm"]:
        idz = [m.actuator("zR").id, m.actuator("zL").id]
    lzs = {"z": [0.0, 0.0], "prev": [0.0, 0.0]}   # the one-way lift's state
    def roll():
        R = d.xmat[1].reshape(3, 3)
        return math.atan2(R[0, 2], R[2, 2])
    # feet touching the floor (user, 05/11: a switch under each foot): the
    # legs' phase set by each landing instead of a fixed clock
    cs = dict(on={"R": True, "L": True}, ph=0.0, f=p["clock_hz"] or 1.5, t_R=-1.0, t_L=-1.0, log=[], NR=1.0, NL=1.0)
    p["_cs"] = cs                      # for a caller to read the landings
    gid = {}
    if p["contact_phase"]:
        gid = {"R": m.geom("Rs").id, "L": m.geom("Ls").id}
    def contacts():
        N = {"R": 0.0, "L": 0.0}
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            for sd in "RL":
                if gid[sd] in (c.geom1, c.geom2):
                    mujoco.mj_contactForce(m, d, i, f6)
                    N[sd] += f6[0]
        return N
    def contact_phase():
        cs["ph"] += cs["f"] * m.opt.timestep
        N = contacts()
        k = m.opt.timestep / 0.03      # smoothed over ~30 ms (contacts chatter)
        for sd in "RL":
            cs["N" + sd] += k * (N[sd] - cs["N" + sd])
        N = {"R": cs["NR"], "L": cs["NL"]}
        tot = N["R"] + N["L"] + 1e-9   # which foot carries the weight: a
        for sd, a in (("R", 0.0), ("L", 0.5)):   # foot "lands" when its share
            was = cs["on"][sd]                   # passes 70 % (after < 30 %)
            now = N[sd] / tot > (0.3 if was else 0.7)
            cs["on"][sd] = now
            if now and not was and d.time - cs["t_" + sd] > 0.25:   # a landing
                cs["t_" + sd] = d.time # (not a bounce): pull the phase towards
                a += p["contact_off"]  # it, a share contact_gain of the error
                err = ((a - cs["ph"]) + 0.5) % 1.0 - 0.5
                cs["ph"] += p["contact_gain"] * err
                cs["f"] = min(2.5, max(0.8, cs["f"] * (1 + p["contact_fgain"] * err)))
                cs["log"].append((round(d.time, 3), sd, round(err, 3), round(cs["f"], 3)))
        return cs["ph"]
    def step():
        th = roll()
        want = cmd[0]
        if p["start_s"] and d.time < p["start_s"] and abs(th) < math.radians(p["start_until_deg"]):
            # the start, from standing: the yaw servo swings on its own at
            # start_hz until the tilt passes start_until_deg (then the loop)
            on_right = (d.time * p["start_hz"]) % 1.0 < 0.5
            want = (1 if on_right else -1) * math.radians(p["start_deg"])
        elif p["clock_hz"]:            # open loop: a fixed clock, no sensor;
            f1, f0, tr = p["clock_hz"], p["clock_hz0"] or p["clock_hz"], p["clock_ramp_s"]
            tt = d.time                # cycles so far: f0 easing to f1 over tr
            cyc = (f0 * tt + (f1 - f0) * tt * tt / (2 * tr) if tr and tt < tr
                   else f1 * tt + ((f0 - f1) * tr / 2 if tr else 0.0))
            if p["clock_b_s"]:         # the rhythm changed on the way (user,
                fb = p["clock_hz"]     # 05/11: quick to stop, calm to walk):
                for ts, fz in ((p["clock_b_s"], p["clock_b"]), (p["clock_c_s"], p["clock_c"])):
                    if ts and d.time >= ts:      # eased over 1 s, integrated
                        fb += (fz - fb) * min(1.0, d.time - ts)
                clock_int[0] += fb * m.opt.timestep
                cyc = clock_int[0]
            on_right = (cyc + p["clock_phase"]) % 1.0 < 0.5
            clock_cyc[0] = cyc
            if p["gyro_rock"]:         # an IMU reads the rocking (user, 10/11):
                dt_ = m.opt.timestep   # its phase (cycles) from roll and roll
                kf = dt_ / 0.01        # rate; the clock locked to it (a PLL)
                gy["w"] += kf * ((th - gy["th"]) / dt_ - gy["w"])
                gy["th"] = th
                w0 = 2 * math.pi * gy["f"]
                gy["amp"] = math.degrees(math.hypot(th, gy["w"] / w0))
                gy["amp_s"] = gy.get("amp_s", 0.0) + dt_ / p["gyro_amp_tau"] * (gy["amp"] - gy.get("amp_s", 0.0))
                phi = math.atan2(-gy["w"] / w0, th) / (2 * math.pi)
                if d.time < p["gyro_s"]:
                    gy["ph"], gy["f"] = cyc, p["clock_hz"]
                else:
                    gy["ph"] += gy["f"] * dt_
                    err = ((phi + p["gyro_off"] - gy["ph"]) + 0.5) % 1.0 - 0.5
                    if gy["amp"] > p["gyro_amp_on"]:
                        if p["gyro_rock"] == 1:
                            gy["ph"] += p["gyro_kp"] * err * dt_
                            gy["f"] = min(3.0, max(1.2, gy["f"] + p["gyro_ki"] * err * dt_))
                        gy["log"].append((d.time, err, gy["f"], gy["amp"]))
                    if p["gyro_rock"] == 1:
                        clock_cyc[0] = gy["ph"]
            if p["contact_phase"]:     # the feet's switches instead
                clock_cyc[0] = contact_phase() - p["clock_phase"]
            g = 1.0                    # the yaw's programme: off (eased 1 s,
            if p["yaw_off_s"] and d.time >= p["yaw_off_s"]:   # or braking:
                tb = d.time - p["yaw_off_s"]           # turned against the
                g = -1.0 if tb < p["yaw_brake_s"] else (  # rocking first)
                    0.0 if p["yaw_brake_s"] else max(0.0, 1.0 - tb))
            if p["yaw_on_s"] and d.time >= p["yaw_on_s"]:
                g = min(1.0, d.time - p["yaw_on_s"])
            want = g * (p["dir"] * T if on_right else -p["dir"] * T)   # first half
            if p["yaw_flip"] and p["turn_s"] and d.time >= p["turn_s"]:
                want = -want               # the yaw's swing reversed from turn_s
            if p["turn_s"] and d.time >= p["turn_s"]:   # turning: the yaw off centre
                want += min(1.0, d.time - p["turn_s"]) * math.radians(p["yaw_bias"])
                if on_right:                            # or bigger one way
                    want *= 1.0 + min(1.0, d.time - p["turn_s"]) * p["yaw_skew"]
        elif th > dead:                #    as if on the right foot
            want = p["dir"] * T
        elif th < -dead:
            want = -p["dir"] * T
        dvp = dv                       # turning by ratchet: one way fast (the
        if p["turn_s"] and d.time >= p["turn_s"] and p["yaw_fast"]:   # feet
            dvp = p["yaw_fast"] * m.opt.timestep   # slip), back slow (they hold)
        if p["yaw_fast"] < 0:
            dvp = dv
        cmd[0] += max(-(dvp if p["yaw_fast_sign"] < 0 else dv), min(dvp if p["yaw_fast_sign"] > 0 else dv, want - cmd[0]))
        d.ctrl[0] = 0.0 if p["feet_pump"] else cmd[0]
        if p["slide_amp"]:             # the slide servo: its own size, speed
            A = p["slide_amp"] / 1000  # and moment (it switches past
            sd_ = math.radians(p["slide_dead"])          # slide_dead)
            ws = scmd[0]
            if p["clock_hz"]:          # open loop too: the same clock, the
                if d.time < p["legs_start_s"]:   # legs late by slide_phase
                    ws = 0.0           # cycles, and only after legs_start_s
                elif p["slide_sine"]:  # a smooth swing: no sudden reversals
                    ph = 2 * math.pi * (clock_cyc[0] + p["clock_phase"] - p["slide_phase"])
                    ramp = min(1.0, (d.time - p["legs_start_s"]) / p["legs_ramp_s"])
                    scmd[0] = -A * ramp * math.sin(ph)   # right back on the right
                    ws = scmd[0]
                else:
                    on_r = (clock_cyc[0] + p["clock_phase"] - p["slide_phase"]) % 1.0 < 0.5
                    ws = -A if on_r else A
            elif th > sd_:
                ws = -A                # on the right foot: right back
            elif th < -sd_:
                ws = A
            scmd[0] += max(-dvs, min(dvs, ws - scmd[0]))
            d.ctrl[1], d.ctrl[2] = scmd[0], -scmd[0]
            if p["clock_hz"] and p["slide_per_leg"] and d.time >= p["legs_start_s"]:
                # a servo per leg (user, 04/11): planted, the leg goes back at
                # a steady 4 A f the whole half cycle; raised, it swings forward
                # and lands already going back at that speed (no jolt)
                ramp = min(1.0, (d.time - p["legs_start_s"]) / p["legs_ramp_s"])
                A_now = A                  # the step's programme (speed changes):
                for key_t, key_a in (("amp_b_s", "amp_b"), ("amp_c_s", "amp_c")):
                    if p[key_t] and d.time >= p[key_t]:   # eased over 1 s
                        u = min(1.0, (d.time - p[key_t]) / p["ease_s"])
                        A_now = A_now + (p[key_a] / 1000 - A_now) * u
                base = clock_cyc[0] + p["clock_phase"] - p["slide_phase"]
                tn = 1.0 if p["turn_s"] and d.time >= p["turn_s"] else 0.0
                tn = min(1.0, d.time - p["turn_s"]) if tn else 0.0   # eased in
                k_now = p["swing_k"]       # the swing's kick, maybe changed on the way
                if p["swing_k_s"] and d.time >= p["swing_k_s"]:
                    k_now += (p["swing_k2"] - k_now) * min(1.0, d.time - p["swing_k_s"])
                dA = tn * p["step_diff"] / 2000    # turning: right step longer
                for j, off, sg in ((1, 0.0, 1.0), (2, 0.5, -1.0)):   # by diff
                    if p["fb_n"] and tn:           # steps forward and back, fb_n
                        c = base + off             # cycles each (user, 05/11)
                        k = math.floor(c)
                        sk = lambda k: 1.0 if (k // int(p["fb_n"])) % 2 == 0 else -1.0
                        Aj = A_now + sg * dA
                        d.ctrl[j] = ramp * leg_profile2(c - k, sk(k) * Aj, sk(k + 1) * Aj)
                    else:
                        d.ctrl[j] = ramp * leg_profile((base + off) % 1.0, A_now + sg * dA, k_now)
                if tn and (p["hold_r"] or p["hold_l"] or p["kick_mm"] or p["osc_r"]):
                    # helping the yaw ratchet with the legs (user, 05/11):
                    # each leg held off centre (its foot then on heel or toe),
                    # and/or the right leg kicked while in the air
                    ph = (clock_cyc[0] + p["clock_phase"]) % 1.0
                    def S(x):
                        x = min(1.0, max(0.0, x))
                        return 3 * x * x - 2 * x ** 3
                    w = p["kick_win"]
                    k0 = p["kick_at"]
                    kick = (S((ph - k0) / w) - S((ph - k0 - w - 0.04) / (0.5 - w - 0.12))) if ph >= k0 else 0.0
                    d.ctrl[1] += tn * (p["hold_r"] + p["kick_mm"] * kick) / 1000
                    d.ctrl[2] += tn * p["hold_l"] / 1000
                    # the right leg stepping forward and back, a sine of
                    # osc_r mm peaking at cycle osc_ph (the left one still)
                    d.ctrl[1] += tn * p["osc_r"] / 1000 * math.cos(2 * math.pi * (ph - p["osc_ph"]))
                if p["spin_mm"] and tn:            # turning on the spot instead
                    ph = clock_cyc[0] + p["clock_phase"]
                    D, w, c = p["spin_mm"] / 1000, p["spin_win"], p["spin_c"]
                    d.ctrl[1] = d.ctrl[1] * (1 - tn) + tn * spin_profile(ph % 1.0, D, w, c)
                    d.ctrl[2] = d.ctrl[2] * (1 - tn) - tn * spin_profile((ph + 0.5) % 1.0, D, w, c)
        if p["foot_yaw"]:              # each foot: turned +foot_yaw deg (to the
            tn = min(1.0, max(0.0, d.time - p["turn_s"])) if p["turn_s"] else 0.0
            if p["turn_e"]:            # turning ends here, eased out over 1 s
                tn *= 1.0 - min(1.0, max(0.0, d.time - p["turn_e"]))
            ph = (clock_cyc[0] + p["clock_phase"]) % 1.0      # left) in the air,
            def S(x):                  # back to straight while planted: the
                x = min(1.0, max(0.0, x))      # body turns over it each step
                return 3 * x * x - 2 * x ** 3
            def fy(q):                 # q: 0..0.5 planted, 0.5..1 in the air
                if q < 0.5:
                    return 1.0 - S((q - 0.05) / 0.4)
                return S((q - 0.55) / 0.4)
            fyd = p["foot_yaw"]        # the turn's programme (a route: user, 07/11)
            for key_t, key_a in (("foot_yaw_b_s", "foot_yaw_b"), ("foot_yaw_c_s", "foot_yaw_c")):
                if p[key_t] and d.time >= p[key_t]:   # eased over 1 s
                    fyd += (p[key_a] - fyd) * min(1.0, d.time - p[key_t])
            dlt = math.radians(fyd) * tn
            if p["yaw_hold"]:          # a gyro holds the heading (user, 10/11):
                dt_ = m.opt.timestep   # its yaw rate (body z, plus a bias) summed
                held["yaw"] = held.get("yaw", 0.0) + (d.qvel[5] + math.radians(p["gyro_bias"])) * dt_
                if d.time >= p["yaw_hold_s"]:  # the feet's turn set by the error
                    e_ = math.degrees(held.get("yaw_ref", 0.0) - held["yaw"])
                    held["yaw_i"] = held.get("yaw_i", 0.0) + e_ * dt_
                    u_ = p["yaw_hold"] * e_ + p["yaw_hold_i"] * held["yaw_i"]
                    u_ = max(-p["yaw_hold_max"], min(p["yaw_hold_max"], u_))
                    dlt += math.radians(p["yaw_sign"] * u_)
                else:
                    held["yaw_ref"] = held["yaw"]
            d.ctrl[3] = dlt * fy(ph)
            d.ctrl[4] = dlt * fy((ph + 0.5) % 1.0)
            if p["feet_pump"]:         # the feet's servos do the yaw too: the
                d.ctrl[3] += cmd[0]    # bar's servo held at 0 and each foot
                d.ctrl[4] += cmd[0]    # turned by what the bar would have
            if p["foot_shared"] == 1:  # one servo, the feet turned opposite
                d.ctrl[3] = dlt * (fy(ph) - 0.5)        # ways (a mirror link)
                d.ctrl[4] = -d.ctrl[3]
            elif p["foot_shared"] == 2:  # one servo, both the same way
                d.ctrl[3] = d.ctrl[4] = dlt * (fy(ph) - 0.5)
        if p["jansen"]:                # Jansen's leg: the crank on the clock,
            fz = p["clock_hz"]         # parked before legs_start_s and after
            q = clock_cyc[0] + p["clock_phase"]          # legs_stop_s
            if d.time < p["legs_start_s"]:
                q = fz * p["legs_start_s"] + p["clock_phase"]
            if p["legs_stop_s"] and d.time >= p["legs_stop_s"]:
                q = fz * p["legs_stop_s"] + p["clock_phase"]
            for j_, off, iz in ((1, 0.0, idz[0]), (2, 0.5, idz[1])):
                n_ = int(((q + off) % 1.0) * len(jb)) % len(jb)
                d.ctrl[j_] = -jb[n_] * js
                d.ctrl[iz] = ju[n_] * js * p["jansen_lift"]
        if p["legs_stop_s"] and d.time >= p["legs_stop_s"]:   # the legs' motor
            if "legs_held" not in held:            # stops: the cams (and the
                held["legs_held"] = (d.ctrl[1], d.ctrl[2])   # legs) stay put
            d.ctrl[1], d.ctrl[2] = held["legs_held"]
        if p["lift_dir_mm"]:           # a one-way lift (user, 09/11): the foot
            for k_ in (0, 1):          # rises while its leg goes forward, stays
                c_ = d.ctrl[1 + k_]    # down while it goes back; the mechanism
                v_ = (c_ - lzs["prev"][k_]) / m.opt.timestep   # takes lift_tau
                lzs["prev"][k_] = c_
                tgt_ = p["lift_dir_mm"] / 1000 if v_ > p["lift_dir_v"] / 1000 else 0.0
                lzs["z"][k_] += (tgt_ - lzs["z"][k_]) * min(1.0, m.opt.timestep / p["lift_tau"])
                d.ctrl[idz[k_]] = lzs["z"][k_]
        if p["ankle_servo"]:           # the ankles (user, 07/11): both feet
            ank = 0.0                  # rolled the same way under the legs,
            if p["ankle_s"] and d.time >= p["ankle_s"]:   # a sine on the clock;
                ank = p["ankle_amp"] * min(1.0, d.time - p["ankle_s"])   # its size
                for key_t, key_a in (("ank_b_s", "ank_b"), ("ank_c_s", "ank_c")):
                    if p[key_t] and d.time >= p[key_t]:   # changed on the way
                        ank += (p[key_a] - ank) * min(1.0, d.time - p[key_t])   # (<0: damps)
            a_ph = 2 * math.pi * (ank_cyc[0] + p["ankle_ph"])
            phi = math.radians(ank) * math.sin(a_ph)
            if p["ankle_fb"]:          # closed loop (user, 07/11): an IMU reads
                # the soles' roll = the body's (IMU) + the ankle's (servo)
                th_ = roll() + 0.5 * (d.qpos[jqa[0]] + d.qpos[jqa[1]])
                kf = m.opt.timestep / 0.01
                imu["w"] += kf * ((th_ - imu["th"]) / m.opt.timestep - imu["w"])
                imu["th"] = th_
                w0 = 2 * math.pi * (p["ankle_f0"] or p["clock_hz"])
                amp_ = math.sqrt(th_ ** 2 + (imu["w"] / w0) ** 2)   # its size now
                tgt = p["ankle_target"]          # the size wanted, maybe changed
                for key_t, key_a in (("ank_t_b_s", "ank_t_b"), ("ank_t_c_s", "ank_t_c")):
                    if p[key_t] and d.time >= p[key_t]:
                        tgt += (p[key_a] - tgt) * min(1.0, d.time - p[key_t])
                on = 1.0 if d.time >= p["ankle_fb_s"] else 0.0
                # rolled with the rocking's speed pumps it, against it damps
                u = p["ankle_g"] * (math.radians(tgt) - amp_) * (imu["w"] / w0) / max(amp_, 0.01)
                if p["ankle_limit"] and amp_ < math.radians(tgt):
                    u = 0.0            # only braking, past the target (a limiter)
                lim = math.radians(p["ankle_max"])
                phi += on * max(-lim, min(lim, u))   # (sign checked in the sim)
            d.ctrl[ida[0]] = d.ctrl[ida[1]] = phi
            ank_cyc[0] += (p["ankle_hz"] or p["clock_hz"]) * m.opt.timestep
        if p["head"]:                  # the head (user, 09/11): on the legs' clock,
            hw = 0.0                   # no sensor; a square kick first, then a
            if p["head_s"] and d.time >= p["head_s"]:   # sine (or square)
                hp_ = clock_cyc[0] + p["clock_phase"] + p["head_ph"]
                if p["head_flip_s"] and d.time >= p["head_flip_s"]:
                    hp_ += 0.5         # out of step: brakes
                if p["head_ph_b_s"] and d.time >= p["head_ph_b_s"]:   # or slid there
                    hp_ += p["head_ph_b"] * min(1.0, (d.time - p["head_ph_b_s"]) / p["head_ph_ease"])
                if p["head_ph_c_s"] and d.time >= p["head_ph_c_s"]:
                    hp_ += p["head_ph_c"] * min(1.0, (d.time - p["head_ph_c_s"]) / p["head_ph_c_ease"])
                if p["head_land_s"] and d.time >= p["head_land_s"]:
                    # the brake timed by the feet's switches (user, 09/11): from
                    # the first right landing after head_land_s (+ a delay) the
                    # phase slides head_ph_b, then the head shrinks
                    if "land_t" not in held:
                        lands = [e[0] for e in cs["log"] if e[0] >= p["head_land_s"] and e[1] == "R"]
                        if lands:
                            held["land_t"] = lands[0] + p["head_land_dly"]
                    if "land_t" in held and d.time >= held["land_t"]:
                        hp_ += p["head_ph_b"] * min(1.0, (d.time - held["land_t"]) / p["head_ph_ease"])
                sq = 1.0 if hp_ % 1.0 < 0.5 else -1.0
                if d.time < p["head_s"] + p["head_kick_s"]:
                    hw = p["head_kick"] * sq
                else:
                    ha = p["head_amp"]
                    if p["head_b_s"] and d.time >= p["head_b_s"]:
                        ha += (p["head_b"] - ha) * min(1.0, (d.time - p["head_b_s"]) / p["ease_s"])
                    if "land_t" in held and d.time >= held["land_t"] + p["head_ph_ease"]:
                        ha *= 1.0 - min(1.0, (d.time - held["land_t"] - p["head_ph_ease"]) / p["ease_s"])
                    if p["gyro_amp_k"] and d.time >= p["gyro_s"]:
                        # the rocking's size held by the head (user, 10/11): the
                        # gyro's size against the target; <0 swings against it
                        tgt = p["gyro_amp_target"]
                        if p["gyro_stop_s"] and d.time >= p["gyro_stop_s"]:
                            tgt *= max(0.0, 1.0 - (d.time - p["gyro_stop_s"]) / p["gyro_stop_ease"])
                        ha = p["head_amp"] * (tgt > 0) + p["gyro_amp_k"] * (tgt - gy.get("amp_s", 0.0))
                        ha = max(-p["head_kick"], min(p["head_kick"], ha))
                    hw = ha * (sq if p["head_wave"] else math.sin(2 * math.pi * hp_))
                if p["head_fb_land"] and d.time >= p["head_land_s"]:
                    # braking step by step with the switches (user, 09/11): at
                    # each landing one half-sine swing of the head, its side set
                    # by which foot landed; no landings (rocking gone): still
                    hw = 0.0
                    ev = [e for e in cs["log"] if e[0] >= p["head_land_s"]]
                    if ev:
                        tl, sd = ev[-1][0], ev[-1][1]
                        Th = p["head_fb_half"] or 0.5 / p["clock_hz"]
                        u = (d.time - tl - p["head_land_dly"]) / Th
                        if 0.0 <= u < 1.0:
                            hw = (1.0 if sd == "R" else -1.0) * p["head_fb_land"] * p["head_fb_amp"] * math.sin(math.pi * u)
                if p["head_brake_s"] and d.time >= p["head_brake_s"]:
                    # one smooth swing against the rocking (user, 09/11: the
                    # jump of the phase flip looked wrong): from the head's next
                    # pass through the middle, half a cycle the other way, then still
                    if "brake_c0" not in held:
                        held["brake_c0"] = math.ceil(hp_ * 2) / 2
                    c0 = held["brake_c0"]
                    if hp_ >= c0 + 0.5:
                        hw = 0.0
                    elif hp_ >= c0:
                        hw = -hw
                if p["head_off_s"] and d.time >= p["head_off_s"]:
                    hw = 0.0
            dh = p["head_speed"] * m.opt.timestep
            if p["head_off_s"] and d.time >= p["head_off_s"] and p["head_off_speed"]:
                dh = p["head_off_speed"] * m.opt.timestep   # back to straight slowly
            hcmd[0] += max(-dh, min(dh, math.radians(hw) - hcmd[0]))
            d.ctrl[idh] = hcmd[0]
        if hook:
            hook(m, d)
        mujoco.mj_step(m, d)
    def heading(i):
        R = d.xmat[i].reshape(3, 3)
        return math.atan2(-R[0, 1], R[1, 1])
    if view:
        from mujoco import viewer
        import time
        with viewer.launch_passive(m, d) as v:
            v.cam.distance, v.cam.elevation, v.cam.azimuth = 0.5, -25, 120
            if p["show_forces"]:             # the floor's push on each ball, as arrows
                v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
                v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
                m.vis.map.force = 0.03
                m.vis.scale.forcewidth = 0.02
            while v.is_running():
                place()
                cmd[0] = hcmd[0] = scmd[0] = clock_int[0] = clock_cyc[0] = ank_cyc[0] = 0.0
                lzs.update(z=[0.0, 0.0], prev=[0.0, 0.0])
                held.clear()           # each replay starts clean (it kept the
                imu.update(th=0.0, w=0.0)   # last run's servos and clocks)
                gy.update(th=0.0, w=0.0, amp=0.0, ph=0.0, f=p["clock_hz"] or 1.5, log=[])
                v.sync()
                time.sleep(1.0)
                wall0 = time.perf_counter()
                while v.is_running() and d.time < p["secs"]:
                    while d.time < (time.perf_counter() - wall0) / p["slow"]:
                        step()
                    v.cam.lookat[:] = [d.qpos[0], d.qpos[1], 0.05]
                    v.sync()
                    time.sleep(0.01)
        return None
    rows = []
    while d.time < p["secs"]:
        step()
        rows.append((d.time, roll(), heading(1), heading(bar), d.qpos[0], d.qpos[1], d.qpos[7]))
    return np.array(rows[::10])


def summary(p, a):
    t, th = a[:, 0], np.degrees(a[:, 1])
    fell = np.abs(th).max() > 30
    late = t > t[-1] - 3
    hb = np.degrees(np.unwrap(a[:, 3]))
    return dict(fell=bool(fell), t_end=float(t[np.argmax(np.abs(th) > 30)]) if fell else float(t[-1]),
                rock=float(np.abs(th[late]).max()), fwd=float((a[-1, 5] - a[0, 5]) * 1000),
                side=float((a[-1, 4] - a[0, 4]) * 1000), turn=float(hb[-1] - hb[0]))


def fmt(s):
    if s["fell"]:
        return f"FELL at {s['t_end']:.1f} s"
    return (f"rocking +-{s['rock']:4.1f} deg, forward {s['fwd']:+6.0f} mm, side {s['side']:+5.0f}, "
            f"turned {s['turn']:+5.0f} deg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--tilt", type=float, default=8.0)
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE")
    a = ap.parse_args()
    p = dict(YAW)
    for kv in a.set:
        k, v = kv.split("=")
        assert k in p, f"unknown: {k}"
        p[k] = float(v)
    if a.view:
        return release(p, math.radians(a.tilt), view=True)
    print(fmt(summary(p, release(p, math.radians(a.tilt)))))


if __name__ == "__main__":
    main()
