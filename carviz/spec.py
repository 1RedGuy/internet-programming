"""Single source of truth for every dimension, tooth count and ratio.

Units: metres, radians, seconds (rpm only where the name says so).
Car frame (Blender world when the car is parked at the origin):
    +X = right (passenger side, LHD car), +Y = forward, +Z = up.
    Origin = ground point directly below the centre of the front axle.

Rotation sign convention used by the drivetrain state (carviz.state):
    Every *scalar* angle the state stores is positive in the "normal running"
    direction:
      * longitudinal members (crank, flywheel, clutch, input/output shafts,
        propshaft, pinion): positive = clockwise seen from the FRONT of the car
        = right-handed rotation about -Y.  (SAE: engine turns counter-clockwise
        seen from the flywheel end.)
      * transverse members (wheels, diff case/ring gear, driveshafts):
        positive = the direction a wheel turns when the car rolls forward
        = right-handed rotation about -X.
    Assemblies convert these scalars to Blender rotations with
    `long_rot(theta)` / `trans_rot(theta)` below.

Nothing in here imports bpy, so it can be unit-tested with plain Python.
"""
from __future__ import annotations

import math

MM = 1e-3
DEG = math.pi / 180.0

# ---------------------------------------------------------------------------
# Video
# ---------------------------------------------------------------------------
FPS = 24
RES_FINAL = (1280, 720)
RES_PREVIEW = (640, 360)

# ---------------------------------------------------------------------------
# Vehicle package (compact front-engine RWD saloon, e.g. BMW 3-series /
# Toyota GT86 class).  Wheelbase/track/tyre are typical for that class.
# ---------------------------------------------------------------------------
WHEELBASE = 2.620
TRACK_FRONT = 1.470
TRACK_REAR = 1.480
Y_FRONT_AXLE = 0.0
Y_REAR_AXLE = -WHEELBASE
BODY_LENGTH = 4.450
BODY_WIDTH = 1.760
BODY_HEIGHT = 1.420
Y_BODY_FRONT = 0.870            # front overhang 0.87 m
Y_BODY_REAR = Y_REAR_AXLE - 0.960  # rear overhang 0.96 m

# Tyre 205/55 R16: OD = 16*25.4 + 2*0.55*205 = 631.9 mm
TYRE_SPEC = "205/55 R16"
RIM_DIAMETER = 16 * 25.4 * MM          # 0.4064
TYRE_WIDTH = 0.205
TYRE_OD = RIM_DIAMETER + 2 * 0.55 * 0.205   # 0.6319
TYRE_RADIUS_UNLOADED = TYRE_OD / 2          # 0.316
ROLLING_RADIUS = 0.305   # loaded dynamic rolling radius used for v = omega*r
WHEEL_CENTER_Z = ROLLING_RADIUS             # wheel centre height (static)
TYRE_MESH_RADIUS = 0.3115   # modelled tyre radius (6.5 mm "squash" hides ground contact)

# ---------------------------------------------------------------------------
# Main drivetrain axis.  Engine, clutch and gearbox share one level axis.
# ---------------------------------------------------------------------------
Z_CRANK = 0.360          # crank / input / output shaft axis height
X_CRANK = 0.0

# ---------------------------------------------------------------------------
# Engine: 2.0 L DOHC 16-valve inline-4, port injected, longitudinal.
# Bore = stroke = 86 mm  ->  4 * pi/4 * 0.086^2 * 0.086 = 1.998 L
# ---------------------------------------------------------------------------
N_CYL = 4
BORE = 86.0 * MM
STROKE = 86.0 * MM
CRANK_THROW = STROKE / 2          # 43 mm
CONROD_LENGTH = 145.0 * MM        # centre-to-centre, rod ratio L/r = 3.37
COMPRESSION_HEIGHT = 31.0 * MM    # pin centre to crown
DECK_HEIGHT = CRANK_THROW + CONROD_LENGTH + COMPRESSION_HEIGHT + 0.8 * MM  # crank axis -> deck
BORE_PITCH = 94.0 * MM            # cylinder centre spacing
Y_CYL1 = 0.060                    # cylinder 1 is at the FRONT
Y_CYL = [Y_CYL1 - i * BORE_PITCH for i in range(N_CYL)]   # [0.060, -0.034, -0.128, -0.222]
Y_BLOCK_FRONT = Y_CYL[0] + 0.070
Y_BLOCK_REAR = Y_CYL[-1] - 0.070
Z_DECK = Z_CRANK + DECK_HEIGHT
DISPLACEMENT_L = N_CYL * math.pi / 4 * BORE**2 * STROKE * 1000
COMPRESSION_RATIO = 10.5
IDLE_RPM = 850
REDLINE_RPM = 6800

# Crank throws (angle of the crankpin at crank angle 0, measured in the
# direction of rotation from top-dead-centre):  1 & 4 together, 2 & 3 together.
CRANKPIN_PHASE_DEG = {1: 0.0, 2: 180.0, 3: 180.0, 4: 0.0}
FIRING_ORDER = (1, 3, 4, 2)
# Crank angle (0..720) at which each cylinder is at TDC *on its power stroke*.
FIRING_TDC_DEG = {1: 0.0, 3: 180.0, 4: 360.0, 2: 540.0}
# Four-stroke windows in each cylinder's own cycle angle phi = (theta - FIRING_TDC) mod 720
STROKES = (("power", 0.0, 180.0), ("exhaust", 180.0, 360.0),
           ("intake", 360.0, 540.0), ("compression", 540.0, 720.0))
# Valve timing in cycle degrees (0 = TDC firing).  Typical road-engine figures:
#   EVO 50 deg BBDC, EVC 10 deg ATDC, IVO 10 deg BTDC, IVC 50 deg ABDC
EVO_DEG, EVC_DEG = 130.0, 370.0       # exhaust open 240 deg of crank
IVO_DEG, IVC_DEG = 350.0, 590.0       # intake  open 240 deg of crank
VALVE_LIFT_INTAKE = 9.5 * MM
VALVE_LIFT_EXHAUST = 9.0 * MM
INTAKE_VALVE_HEAD_D = 33.0 * MM
EXHAUST_VALVE_HEAD_D = 28.0 * MM
VALVES_PER_CYL = 4
SPARK_ADVANCE_DEG = 15.0   # spark ~15 deg before TDC-firing (cycle angle 705)

# Valvetrain drive: duplex/roller timing chain, 2:1 reduction.
CRANK_SPROCKET_TEETH = 21
CAM_SPROCKET_TEETH = 42
CHAIN_PITCH = 9.525 * MM   # 3/8 inch
CAM_SPEED_RATIO = CRANK_SPROCKET_TEETH / CAM_SPROCKET_TEETH   # 0.5 exactly
CAM_CENTRE_SPACING = 0.13552  # intake <-> exhaust cam centres: smallest spacing at which two 42T
                              # 3/8" sprockets clear and the chain closes on a whole number (126) of links

# Flywheel (dual-mass ignored: single-mass, with starter ring gear)
FLYWHEEL_DIAMETER = 0.300
FLYWHEEL_THICKNESS = 0.030
FLYWHEEL_RING_TEETH = 132
Y_CRANK_FLANGE = Y_BLOCK_REAR - 0.008          # rear crank flange face
Y_FLYWHEEL_FRONT = Y_CRANK_FLANGE
Y_FLYWHEEL_FACE = Y_FLYWHEEL_FRONT - FLYWHEEL_THICKNESS   # rear (friction) face

# ---------------------------------------------------------------------------
# Clutch: single dry plate, push-type diaphragm spring, hydraulic release.
# ---------------------------------------------------------------------------
CLUTCH_DISC_OD = 0.228          # 9 in class
CLUTCH_DISC_ID = 0.150          # facing inner diameter
CLUTCH_FACING_THICKNESS = 3.5 * MM
CLUTCH_DISC_THICKNESS = 8.4 * MM     # clamped, incl. cushion segments
CLUTCH_DISC_SPLINE_TEETH = 23
CLUTCH_DAMPER_SPRINGS = 6
PRESSURE_PLATE_OD = 0.232
PRESSURE_PLATE_THICKNESS = 16.0 * MM
CLUTCH_COVER_OD = 0.280
DIAPHRAGM_FINGERS = 18
PRESSURE_PLATE_LIFT = 1.8 * MM       # at full release
RELEASE_BEARING_TRAVEL = 9.0 * MM    # at full pedal (finger-tip travel)
DIAPHRAGM_LEVER_RATIO = RELEASE_BEARING_TRAVEL / PRESSURE_PLATE_LIFT  # = 5.0 (nominal overall; as built the lever is 4.24 while the plate lifts — do not label 5.0)
Y_DISC_CENTRE = Y_FLYWHEEL_FACE - CLUTCH_DISC_THICKNESS / 2

# Hydraulic release: incompressible fluid -> A_m * x_m = A_s * x_s
CLUTCH_PEDAL_TRAVEL = 0.140          # at the pedal pad
CLUTCH_PEDAL_RATIO = 6.0             # pad travel : master pushrod travel
MASTER_CYL_BORE = 15.87 * MM         # 5/8 in
SLAVE_CYL_BORE = 19.05 * MM          # 3/4 in
MASTER_STROKE = CLUTCH_PEDAL_TRAVEL / CLUTCH_PEDAL_RATIO                 # 23.3 mm
SLAVE_STROKE = MASTER_STROKE * (MASTER_CYL_BORE / SLAVE_CYL_BORE) ** 2   # 16.2 mm incl. free play (SUPERSEDED: use SLAVE_WORKING_STROKE 14.9 mm)
RELEASE_FORK_RATIO = SLAVE_STROKE / RELEASE_BEARING_TRAVEL               # 1.8 (SUPERSEDED: as built RELEASE_FORK_RATIO_EFFECTIVE = 1.655)
CLUTCH_FREE_PLAY = 0.08     # fraction of pedal travel before bearing moves
CLUTCH_BITE_LO = 0.22       # pedal fraction where torque capacity starts to fall
CLUTCH_BITE_HI = 0.50       # pedal fraction where clutch is fully released

# ---------------------------------------------------------------------------
# Gearbox: 5-speed, 3-shaft (input / countershaft / output), constant mesh,
# synchronised.  Input and output shafts are coaxial; 4th is direct drive.
# All forward pairs: helical, normal module 2.25, helix 25 deg, normal PA 20,
# common tooth sum 61 so every pair shares the same centre distance.
# ---------------------------------------------------------------------------
GEAR_NORMAL_MODULE = 2.25 * MM
GEAR_HELIX = 25.0 * DEG
GEAR_PRESSURE_ANGLE = 20.0 * DEG
GEAR_TRANSVERSE_MODULE = GEAR_NORMAL_MODULE / math.cos(GEAR_HELIX)
GEAR_TOOTH_SUM = 61
GEARBOX_CENTRE_DISTANCE = GEAR_TRANSVERSE_MODULE * GEAR_TOOTH_SUM / 2   # 75.72 mm
Z_COUNTERSHAFT = Z_CRANK - GEARBOX_CENTRE_DISTANCE    # countershaft directly below

# (input/countershaft constant-mesh "headset")
Z_INPUT = 26
Z_CS_DRIVEN = 35
HEADSET_RATIO = Z_CS_DRIVEN / Z_INPUT          # 1.346
# forward pairs: gear -> (countershaft teeth, output-shaft teeth)
GEAR_PAIRS = {1: (17, 44), 2: (24, 37), 3: (30, 31), 5: (38, 23)}
# reverse: spur gears, module 2.5, countershaft -> idler -> output gear
REV_MODULE = 2.5 * MM
Z_REV_CS = 15
Z_REV_IDLER = 22
Z_REV_OUT = 38


def gear_ratio(g):
    """Overall gearbox ratio  (input rpm / output rpm), signed (reverse < 0)."""
    if g in (None, 0, "N"):
        return None
    if g == 4:
        return 1.0
    if g == "R":
        return -HEADSET_RATIO * Z_REV_OUT / Z_REV_CS
    zc, zm = GEAR_PAIRS[g]
    return HEADSET_RATIO * zm / zc


GEAR_RATIOS = {g: gear_ratio(g) for g in (1, 2, 3, 4, 5, "R")}
# -> 1: 3.484  2: 2.075  3: 1.391  4: 1.000  5: 0.815  R: -3.410

# Synchronisers: single-cone, brass blocker rings.
# Sleeve position s in [-1, +1]; physical axial offset = s * SLEEVE_TRAVEL along +Y
# (s = +1 -> sleeve moved FORWARD, s = -1 -> sleeve moved REARWARD).
SYNCHROS = {            # name: (gear when sleeve moves REARWARD (-Y), gear when FORWARD (+Y))
    "12": (1, 2),       # 1-2 synchro sits between 2nd (front) and 1st (rear)
    "34": (3, 4),       # 3-4 synchro: 4th = input gear dog teeth (front), 3rd behind
    "5R": (5, "R"),     # 5-R synchro: reverse gear (front), 5th behind
}
SYNCHRO_GEARS = {k: {-1: v[0], +1: v[1]} for k, v in SYNCHROS.items()}
GEAR_SYNCHRO = {g: (k, side) for k, d in SYNCHRO_GEARS.items() for side, g in d.items()}
BLOCKER_INDEX = 0.25    # blocker ring index travel, fraction of one dog-tooth pitch
SLEEVE_TRAVEL = 8.5 * MM        # neutral -> fully engaged, each way
DOG_TEETH = 32                  # dog (clutch) teeth on each gear / sleeve spline count
# Sleeve stroke phases (fraction of SLEEVE_TRAVEL) - shared by state + gearbox model
SYNC_CONTACT = 0.30     # struts have pushed blocker ring onto the cone
SYNC_BLOCK = 0.45       # sleeve chamfers bear on blocker-ring chamfers (held here while syncing)
SYNC_THROUGH = 0.72     # sleeve has passed through blocker ring teeth, touching dog chamfers
SYNC_ENGAGED = 0.95     # dogs fully engaged (>= this counts as "in gear")

# Shift lever: 5-speed H-pattern with reverse bottom-right
#    1   3   5
#    |---|---|
#    2   4   R
SHIFT_GATE = {1: (-1, +1), 2: (-1, -1), 3: (0, +1), 4: (0, -1), 5: (+1, +1), "R": (+1, -1),
              "N": (0, 0)}
LEVER_GATE_SPACING = 0.030     # knob lateral travel per plane (approx)
LEVER_THROW = 0.055            # knob fore/aft travel to engage (approx)

# Gearbox axial layout (Y in car frame).  Owned by gearbox assembly; listed
# here so other assemblies (bellhousing, propshaft) agree.
Y_BELLHOUSING_FRONT = Y_BLOCK_REAR
Y_GEARBOX_FRONT = -0.470       # bellhousing / main case joint
Y_GEARBOX_REAR = -1.120        # output flange face
Y_SHIFT_LEVER = -0.960         # lever pivot Y (direct linkage on top of the box)

# ---------------------------------------------------------------------------
# Propshaft (one-piece tube, Hooke joint each end, Z-arrangement so the
# velocity fluctuation of the two joints cancels)
# ---------------------------------------------------------------------------
PROPSHAFT_TUBE_D = 0.065
Y_PINION_FLANGE = -2.400
Z_PINION = WHEEL_CENTER_Z        # pinion axis on the ring-gear centre line (no hypoid offset)

# ---------------------------------------------------------------------------
# Final drive + open differential
# ---------------------------------------------------------------------------
Z_RING = 41
Z_PINION_TEETH = 10
FINAL_DRIVE = Z_RING / Z_PINION_TEETH      # 4.10
RING_PITCH_DIAMETER = 0.190                # 7.5 in axle class
Z_SIDE_GEAR = 16
Z_SPIDER = 10
N_SPIDERS = 2
Y_DIFF = Y_REAR_AXLE
Z_DIFF = WHEEL_CENTER_Z

# ---------------------------------------------------------------------------
# Driveshafts (half shafts): plunging tripod inner joint, Rzeppa outer joint
# ---------------------------------------------------------------------------
HALFSHAFT_D = 0.025
X_DIFF_OUTPUT = 0.150        # diff output-flange outer face at +-X (the tripod housing bolts on here)
X_WHEEL_HUB = TRACK_REAR / 2 - 0.085   # outer joint centre (inboard of wheel centre plane)
RZEPPA_BALLS = 6
TRIPOD_ROLLERS = 3
SUSPENSION_TRAVEL = 0.060    # +- wheel centre travel shown in scene 7

# ---------------------------------------------------------------------------
# Derived helpers (pure python)
# ---------------------------------------------------------------------------

def rpm_to_rad_s(rpm):
    return rpm * 2 * math.pi / 60.0


def rad_s_to_rpm(w):
    return w * 60.0 / (2 * math.pi)


def road_speed_kmh(engine_rpm, gear):
    """Road speed for a given engine speed with the clutch locked in a gear."""
    r = gear_ratio(gear)
    w_wheel = rpm_to_rad_s(engine_rpm) / (r * FINAL_DRIVE)
    return w_wheel * ROLLING_RADIUS * 3.6


def engine_rpm_at(speed_kmh, gear):
    r = gear_ratio(gear)
    w_wheel = speed_kmh / 3.6 / ROLLING_RADIUS
    return rad_s_to_rpm(w_wheel * r * FINAL_DRIVE)


def long_rot(theta):
    """Blender Euler (x, y, z) for a longitudinal member turned by scalar theta."""
    return (0.0, -theta, 0.0)


def trans_rot(theta):
    """Blender Euler (x, y, z) for a transverse member turned by scalar theta."""
    return (-theta, 0.0, 0.0)


if __name__ == "__main__":  # quick self-report
    print(f"displacement {DISPLACEMENT_L:.3f} L, deck height {DECK_HEIGHT*1000:.1f} mm")
    print(f"centre distance {GEARBOX_CENTRE_DISTANCE*1000:.2f} mm")
    for g, r in GEAR_RATIOS.items():
        extra = "" if g == "R" else f"  3000 rpm -> {road_speed_kmh(3000, g):6.1f} km/h"
        print(f"gear {g}: {r:+.3f}{extra}")
    print(f"final drive {FINAL_DRIVE:.3f}; slave stroke {SLAVE_STROKE*1000:.1f} mm; fork ratio {RELEASE_FORK_RATIO:.2f}")
    print("1->2 at 3000 rpm lands at", round(3000 * GEAR_RATIOS[2] / GEAR_RATIOS[1]), "rpm;",
          "2->3 at 3000 rpm lands at", round(3000 * GEAR_RATIOS[3] / GEAR_RATIOS[2]), "rpm")


# ---------------------------------------------------------------------------
# Cabin / pedal box / clutch hydraulics layout (LHD).  Shared by the clutch,
# body and engine assemblies so nothing collides.  (appended)
# ---------------------------------------------------------------------------
X_DRIVER = -0.370                 # steering-column / driver centreline
Y_FIREWALL = -0.450               # firewall (engine-bay side face) at pedal-box height
Z_FLOOR = 0.200                   # cabin floor (footwell) height
CLUTCH_PEDAL_PIVOT = (-0.505, -0.560, 0.700)   # pedal-box pivot (hanging pedals)
PEDAL_ARM = 0.300                 # pivot -> pad centre (matches kin.clutch_geometry default)
PEDAL_REST_ANGLE = 0.38           # rad: arm leans rearward from vertical at rest (pad behind pivot)
BRAKE_PEDAL_X = -0.385
THROTTLE_PEDAL_X = -0.265
MASTER_CYL_POS = (-0.505, -0.420, 0.650)       # master cylinder body centre (engine side of firewall; as built, 6:1 pedal lever)
MASTER_CYL_AXIS = (0.0, 1.0, 0.0)              # pushrod pushes forward (+Y) into the master cylinder
SLAVE_CYL_POS = (-0.1851, -0.415, 0.360)       # external slave on the bellhousing, left side (as built: outside the wall)
RELEASE_FORK_PIVOT = (-0.0697, -0.4092, 0.360) # ball stud inside the bellhousing (as built: exact 1.655 fork ratio)
Y_RELEASE_BEARING = -0.3751                    # release bearing face at rest (as built)
H_POINT = (X_DRIVER, -1.520, 0.420)            # driver hip point
STEERING_WHEEL_CENTRE = (X_DRIVER, -1.020, 0.900)
SHIFT_KNOB_REST = (0.0, Y_SHIFT_LEVER - 0.040, 0.880)

# Clutch free play sits between pedal pushrod and master piston (appended):
# the hydraulics only move once the free play is taken up, and the release
# bearing (self-adjusting hydraulic release) is always in light contact with
# the fingers, so every mm of slave travel moves the bearing.
MASTER_WORKING_STROKE = CLUTCH_PEDAL_TRAVEL * (1 - CLUTCH_FREE_PLAY) / CLUTCH_PEDAL_RATIO      # 21.5 mm
SLAVE_WORKING_STROKE = MASTER_WORKING_STROKE * (MASTER_CYL_BORE / SLAVE_CYL_BORE) ** 2       # 14.9 mm
RELEASE_FORK_RATIO_EFFECTIVE = SLAVE_WORKING_STROKE / RELEASE_BEARING_TRAVEL                  # 1.655

# Flat (bucket) tappet three-arc cams (appended): base circle and nose radius.
CAM_BASE_RADIUS = 18.0 * MM
CAM_NOSE_RADIUS = 5.0 * MM

# Clutch-disc / input-shaft splines (appended; shared by clutch and gearbox)
CLUTCH_SPLINE_D_MAJOR = 25.4 * MM
CLUTCH_SPLINE_D_MINOR = 21.5 * MM

# As built (wheels assembly, inner_joint='flange'): tripod joint centre sits 27 mm
# outboard of the diff output flange.  Joint spacing 0.478 m; plunge 3.78 mm and
# joint angle 7.2 deg at +-60 mm wheel travel.
X_INNER_JOINT_CENTRE = 0.1771
