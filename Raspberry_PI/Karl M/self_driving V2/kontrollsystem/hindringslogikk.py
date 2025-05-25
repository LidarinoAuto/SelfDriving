from sensorer import lidar, ultrasound
import time
import math
import logg
import visning.visualisering as vis

# Robot dimensions and safety
ROBOT_DIAMETER = 23.5  # cm
SAFETY_MARGIN = 2.0    # cm
SIKKERHETS_RADIUS = ROBOT_DIAMETER / 2 + SAFETY_MARGIN  # cm

# Control parameters
STEP = 100
ROTATE_STEP_DEFAULT = 50.0
HEADING_TOLERANCE = 18.0   # degrees
DEADZONE = 8.0             # �kt for � unng� frem/tilbake
DRIVE_TIME = 2.0           # seconds
SAFE_DISTANCE_ULTRASOUND = 10.0  # cm
MAX_SEARCH_ROTATE_TIME = 4.0     # sekunder
MIN_OPENING_WIDTH = 25           # Minimum bredde p� �pning i grader

# State variables
state = "IDLE"
drive_start_time = None
current_target_angle = None
search_start_time = None

def heading_difference(target, current):
    """Returnerer korteste vinkel-differanse fra current til target i grader (-180 til 180)."""
    diff = (target - current + 180) % 360 - 180
    return diff

def heading_correction(current_heading, target_heading, k=0.12, max_omega=ROTATE_STEP_DEFAULT):
    """Proportional heading correction, clamped to max_omega."""
    delta = heading_difference(target_heading, current_heading)
    omega = k * delta
    return max(-max_omega, min(max_omega, omega))

def finn_aapninger():
    """Returner liste av (start, end, width) for �pne segmenter."""
    fri = [False] * 360
    for angle, distance in lidar.scan_data:
        if not isinstance(angle, (int, float)) or distance <= SIKKERHETS_RADIUS:
            continue
        delta = math.degrees(math.asin(min(SIKKERHETS_RADIUS / distance, 1.0)))
        start = int(math.floor(angle - delta)) % 360
        end = int(math.ceil(angle + delta)) % 360
        i = start
        while True:
            fri[i] = True
            if i == end:
                break
            i = (i + 1) % 360

    segments = []
    i = 0
    while i < 360:
        if fri[i]:
            start = i
            while fri[i % 360]:
                i += 1
            end = (i - 1) % 360
            width = (end - start + 1) if end >= start else (360 - start + end + 1)
            segments.append((start, end, width))
        else:
            i += 1

    # debug logging
    total_free = sum(fri)
    logg.skriv_logg(f"[DEBUG] Antall frie grader: {total_free}, segmenter funnet: {len(segments)}")

    # wraparound merge
    if segments and segments[0][0] == 0 and segments[-1][1] == 359:
        first = segments.pop(0)
        last = segments.pop(-1)
        merged = (last[0], first[1], first[2] + last[2])
        segments.insert(0, merged)

    # Filtrer ut �pninger som er for smale
    wide_segments = [s for s in segments if s[2] >= MIN_OPENING_WIDTH]
    return wide_segments

def finn_storste_aapning():
    """Return center angle of the largest opening, or None if none."""
    openings = finn_aapninger()
    if not openings:
        logg.skriv_logg("[DEBUG] Ingen �pning: ingen segmenter tilstrekkelig brede.")
        vis.sett_aapninger([], None)
        return None

    segment_pairs = [(start, end) for start, end, _ in openings]
    best = max(openings, key=lambda x: x[2])
    start, end, width = best
    center = (start + width / 2) % 360
    logg.skriv_logg(f"[S�K] Valgt �pning {start}?{end}, bredde {width}, midt: {center:.1f}")
    vis.sett_aapninger(segment_pairs, center)
    return center

def ultralyd_blokkert():
    """Return True hvis noen ultralydsensor ser hindring n�rt."""
    for sensor, dist in ultrasound.sensor_distances.items():
        if 0 < dist < SAFE_DISTANCE_ULTRASOUND:
            logg.skriv_logg(f"[ULTRALYD] Sensor {sensor} blokkert: {dist:.1f} cm")
            return True
    return False

def autonom_logikk(heading):
    """Robust state machine for obstacle avoidance."""
    global state, drive_start_time, current_target_angle, search_start_time

    if state == "IDLE":
        if ultralyd_blokkert() or not lidar.is_path_clear():
            state = "SEARCH"
            current_target_angle = None
            search_start_time = time.time()
            logg.skriv_logg("[HINDRING] Hindring oppdaget, starter s�k.")
            return (0, 0, ROTATE_STEP_DEFAULT)
        return (STEP, 0, 0)

    if state == "SEARCH":
        # Timeout? Kj�r frem uansett!
        if search_start_time and (time.time() - search_start_time) > MAX_SEARCH_ROTATE_TIME:
            state = "DRIVING"
            drive_start_time = time.time()
            logg.skriv_logg("[S�K] Timeout p� rotasjon, pr�ver � kj�re frem.")
            current_target_angle = None
            return (STEP, 0, 0)

        # Hvis vi ikke har valgt m�l, velg st�rste �pning �n gang
        if current_target_angle is None:
            angle = finn_storste_aapning()
            if angle is None:
                logg.skriv_logg("[S�K] Ingen �pning funnet, roterer videre.")
                return (0, 0, ROTATE_STEP_DEFAULT)
            current_target_angle = angle
            logg.skriv_logg(f"[S�K] Target for �pning: {current_target_angle}")

        # Hold p� valgt m�l helt til vi peker mot det
        delta = heading_difference(current_target_angle, heading)
        logg.skriv_logg(f"[S�K] Mot �pning: target={current_target_angle:.1f}, n�={heading:.1f}, delta={delta:.1f}")

        if abs(delta) < DEADZONE:
            omega = 0
            logg.skriv_logg("[S�K] D�dssone aktivert, omega=0")
        else:
            omega = heading_correction(heading, current_target_angle)
            logg.skriv_logg(f"[S�K] Justerer, omega={omega:.1f}")

        # Hvis vi er "innenfor" HEADING_TOLERANCE, g� over til kj�ring!
        if abs(delta) <= HEADING_TOLERANCE:
            state = "DRIVING"
            drive_start_time = time.time()
            logg.skriv_logg(f"[S�K] Justering mot {current_target_angle:.1f}� ferdig, kj�rer frem.")
            current_target_angle = None
            return (STEP, 0, 0)
        else:
            return (0, 0, omega)

    if state == "DRIVING":
        if ultralyd_blokkert():
            logg.skriv_logg("[DRIVING] Hindring foran, avbryter.")
            state = "IDLE"
            current_target_angle = None
            return (0, 0, 0)
        if (time.time() - drive_start_time) < DRIVE_TIME:
            return (STEP, 0, 0)
        logg.skriv_logg("[DRIVING] Kj�ring ferdig, tilbake til IDLE.")
        state = "IDLE"
        current_target_angle = None
        return (0, 0, 0)
