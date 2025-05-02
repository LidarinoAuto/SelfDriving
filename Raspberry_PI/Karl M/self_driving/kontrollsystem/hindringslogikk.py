from sensorer import lidar, ultrasound
import time
import math
import logg
import visning.visualisering as vis

# Robot dimensions and safety
ROBOT_DIAMETER = 23.5  # cm
SAFETY_MARGIN = 2.0     # cm
SIKKERHETS_RADIUS = ROBOT_DIAMETER / 2 + SAFETY_MARGIN  # cm

# Control parameters
STEP = 100
ROTATE_STEP_DEFAULT = 50.0
HEADING_TOLERANCE = 5.0   # degrees
DRIVE_TIME = 2.0          # seconds
SAFE_DISTANCE_ULTRASOUND = 10.0  # cm

# State variables
state = "IDLE"
drive_start_time = None
current_target_angle = None


def normalize_angle(angle):
    """Normalize angle to [-180, 180)."""
    return (angle + 180) % 360 - 180


def heading_correction(current_heading, target_heading, k=0.5, max_omega=ROTATE_STEP_DEFAULT):
    """Proportional heading correction, clamped to max_omega."""
    delta = normalize_angle(target_heading - current_heading)
    omega = k * delta
    return max(-max_omega, min(max_omega, omega))


def finn_aapninger():
    """Return list of (start, end, width) for free angular segments."""
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

    return segments


def finn_storste_aapning():
    """Return center angle of the largest opening, or None if none, with debug logs."""
    openings = finn_aapninger()
    if not openings:
        logg.skriv_logg(f"[DEBUG] Ingen �pning: ingen segmenter tilstrekkelig brede.")
        vis.sett_aapninger([], None)
        return None

    # convert for visualization
    segment_pairs = [(start, end) for start, end, _ in openings]
    best = max(openings, key=lambda x: x[2])
    start, end, width = best
    center = (start + width / 2) % 360
    logg.skriv_logg(f"[S�K] Valgt �pning {start}�?{end}�, bredde {width}�, midt: {center:.1f}�")
    vis.sett_aapninger(segment_pairs, center)
    return center


def ultralyd_blokkert():
    """Return True if any ultrasound sensor detects an obstacle within threshold."""
    for sensor, dist in ultrasound.sensor_distances.items():
        if 0 < dist < SAFE_DISTANCE_ULTRASOUND:
            logg.skriv_logg(f"[ULTRALYD] Sensor {sensor} blokkert: {dist:.1f} cm")
            return True
    return False


def autonom_logikk(heading):
    """Main obstacle avoidance state machine with clear search fallback."""
    global state, drive_start_time, current_target_angle

    if state == "IDLE":
        if ultralyd_blokkert() or not lidar.is_path_clear():
            state = "SEARCH"
            logg.skriv_logg("[HINDRING] Hindring oppdaget, starter s�k.")
            return (0, 0, ROTATE_STEP_DEFAULT)
        return (STEP, 0, 0)

    if state == "SEARCH":
        angle = finn_storste_aapning()
        if angle is None:
            logg.skriv_logg("[S�K] Ingen �pning funnet, roterer 30� videre.")
            return (0, 0, 30.0)
        current_target_angle = angle
        drive_start_time = None
        state = "DRIVING"
        logg.skriv_logg(f"[S�K] �pning ved {angle:.1f}�, justerer heading.")
        return (0, 0, heading_correction(heading, current_target_angle))

    if state == "DRIVING":
        if ultralyd_blokkert():
            logg.skriv_logg("[DRIVING] Hindring foran, avbryter.")
            state = "IDLE"
            return (0, 0, 0)

        delta = normalize_angle(current_target_angle - heading)
        if abs(delta) > HEADING_TOLERANCE:
            logg.skriv_logg(f"[DRIVING] Justerer heading: delta {delta:.1f}�")
            return (0, 0, heading_correction(heading, current_target_angle))

        if drive_start_time is None:
            drive_start_time = time.time()
            logg.skriv_logg("[DRIVING] Heading ok, kj�rer frem.")

        if (time.time() - drive_start_time) < DRIVE_TIME:
            return (STEP, 0, 0)

        logg.skriv_logg("[DRIVING] Kj�ring ferdig, tilbake til IDLE.")
        state = "IDLE"
        return (0, 0, 0)
