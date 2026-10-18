import argparse, queue, re, sys, threading, time, urllib.request, urllib.parse
import cv2, numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG  ← edit these two lines to match your hardware before running
# ─────────────────────────────────────────────────────────────────────────────
#   ESP32-CAM IP: check the Serial Monitor after flashing esp32cam_wifi_stream.ino
#   Arduino port: /dev/ttyUSB0 or /dev/ttyACM0 on Pi; COM3 etc. on Windows
# ─────────────────────────────────────────────────────────────────────────────
ap = argparse.ArgumentParser()
ap.add_argument('--model', default='yolo11s.onnx', help='falls back to yolo11n.onnx if this file is missing')
ap.add_argument('--conf', type=float, default=0.30)
ap.add_argument('--size', type=int, default=320)
ap.add_argument('--ard-port', default=None,
                help='(unused) Arduino is now bridged via ESP32-CAM HTTP – no direct serial')
ap.add_argument('--ard-baud', type=int, default=115200, help='(unused)')
ap.add_argument('--cam-url', default='http://esp32cam.local:81/stream',
                help='ESP32-CAM MJPEG stream URL')
ap.add_argument('--speed', type=int, default=190, help='forward speed 0-255')
ap.add_argument('--turn',  type=int, default=175, help='turning speed 0-255')
ap.add_argument('--no-auto-flash', action='store_true',
                help='start with the flash LED forced off (battery-safe default)')
ap.add_argument('--close', type=float, default=1.0,
                help='stop when the target box is this fraction of the frame height. Higher = rover gets closer (1.0 = box as tall as the picture)')
ap.add_argument('--slow', type=int, default=140,
                help='forward speed 0-255 for the final approach (lower = more precise stop)')
ap.add_argument('--turn-rate', type=float, default=90.0,
                help='how fast the rover pivots at --turn speed, in degrees per second (measure it, see notes)')
ap.add_argument('--drive-cm-s', type=float, default=25.0,
                help='how fast the rover drives at --speed, in cm per second (measure it, see notes)')
ap.add_argument('--scan-step', type=float, default=30.0, help='degrees per step while scanning the room')
ap.add_argument('--flip', action='store_true', help='rotate the picture 180 degrees')
ap.add_argument('--dry-run', action='store_true',
                help='print motor commands instead of sending them (no Arduino needed)')
# parse_known_args lets Flask/Gunicorn pass their own flags without crashing
args = ap.parse_known_args()[0]

CLOSE_FRAC = args.close          # target box this tall (fraction of frame) = close enough
ALIGN = 0.12               # box centre within +-12% of frame centre = "aligned"

NAMES = ("person bicycle car motorcycle airplane bus train truck boat traffic_light fire_hydrant "
 "stop_sign parking_meter bench bird cat dog horse sheep cow elephant bear zebra giraffe backpack "
 "umbrella handbag tie suitcase frisbee skis snowboard sports_ball kite baseball_bat baseball_glove "
 "skateboard surfboard tennis_racket bottle wine_glass cup fork knife spoon bowl banana apple "
 "sandwich orange broccoli carrot hot_dog pizza donut cake chair couch potted_plant bed "
 "dining_table toilet tv laptop mouse remote keyboard cell_phone microwave oven toaster sink "
 "refrigerator book clock vase scissors teddy_bear hair_drier toothbrush").split()
ALIASES = {'human': 'person', 'man': 'person', 'woman': 'person', 'me': 'person', 'people': 'person',
 'phone': 'cell_phone', 'mobile': 'cell_phone', 'plant': 'potted_plant', 'table': 'dining_table',
 'sofa': 'couch', 'television': 'tv', 'water_bottle': 'bottle', 'mug': 'cup', 'ball': 'sports_ball',
 'bag': 'backpack', 'controller': 'remote', 'teddy': 'teddy_bear'}

def norm(text):
    s = text.strip().lower().replace(' ', '_')
    for p in ('a_', 'an_', 'the_', 'my_'):
        if s.startswith(p): s = s[len(p):]
    return ALIASES.get(s, s)

# ---------------- AI model (YOLO11n via OpenCV, runs locally) ----------------
cv2.setNumThreads(3)                                 # leave one core free
import os
_here = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = None
for _name in (args.model, 'yolo11n.onnx'):      # wanted model first, then the old nano model
    for _p in (_name, os.path.join(_here, _name)):
        if os.path.exists(_p):
            MODEL_PATH = _p; break
    if MODEL_PATH: break
MODEL_PATH = MODEL_PATH or args.model
try:
    net = cv2.dnn.readNetFromONNX(MODEL_PATH)
    print(f"Loaded YOLO model: {MODEL_PATH}")
except Exception as e:
    net = None
    print(f"WARNING: Could not load '{MODEL_PATH}' ({e}). NO DETECTIONS - stream-only mode.")

# YOLO ONNX files have a fixed input size (usually 640, or whatever you exported with).
# Test-run the model once and pick the size that works, so a wrong --size can't silently break detection.
if net is not None:
    for _sz in dict.fromkeys([args.size, 640, 320]):
        try:
            net.setInput(cv2.dnn.blobFromImage(np.zeros((_sz, _sz, 3), np.uint8), 1 / 255.0, (_sz, _sz), swapRB=True))
            net.forward()
            args.size = _sz
            print(f"Model test run OK, input size {_sz}x{_sz}")
            break
        except Exception as e:
            print(f"Model test at {_sz}x{_sz} failed: {str(e)[:120]}")
    else:
        print("WARNING: model failed every test run - detection disabled. Re-export yolo11n.onnx (see notes).")
        net = None

diag = {'model': (f"{os.path.basename(MODEL_PATH)} @{args.size}" if net is not None else 'NOT LOADED'),
        'raw_max': 0.0, 'raw_label': '-', 'n': 0, 'err': ''}

def letterbox(img, size):
    h, w = img.shape[:2]
    s = size / max(h, w)
    nh, nw = int(round(h * s)), int(round(w * s))
    canvas = np.full((size, size, 3), 114, np.uint8)
    top, left = (size - nh) // 2, (size - nw) // 2
    canvas[top:top + nh, left:left + nw] = cv2.resize(img, (nw, nh))
    return canvas, s, left, top

def detect(img):
    """Only ever called from the Vision thread (cv2.dnn nets are not thread-safe)."""
    if net is None:
        return []
    canvas, s, left, top = letterbox(img, args.size)
    net.setInput(cv2.dnn.blobFromImage(canvas, 1 / 255.0, (args.size, args.size), swapRB=True))
    out = net.forward()[0].T
    scores = out[:, 4:]
    cls, conf = scores.argmax(1), scores.max(1)
    _i = int(conf.argmax())
    diag['raw_max'], diag['raw_label'] = float(conf[_i]), NAMES[int(cls[_i])]
    keep = conf > args.conf
    out, cls, conf = out[keep], cls[keep], conf[keep]
    boxes = [[float((o[0] - o[2] / 2 - left) / s), float((o[1] - o[3] / 2 - top) / s),
              float(o[2] / s), float(o[3] / s)] for o in out]
    if not boxes: return []
    idx = np.array(cv2.dnn.NMSBoxes(boxes, conf.tolist(), args.conf, 0.45)).flatten()
    return [(NAMES[cls[i]], float(conf[i]), boxes[i]) for i in idx]

def pick(dets, label):
    c = [d[2] for d in dets if d[0] == label]
    return max(c, key=lambda b: b[2] * b[3]) if c else None      # biggest = closest

# ---------------- Arduino link: HTTP bridge via the ESP32-CAM ----------------
class Arduino:
    """
    Motor commands are relayed from the Pi to ESP32-CAM over WiFi,
    and forwarded by ESP32-CAM to Arduino over UART:
      Pi  ──WiFi──►  ESP32-CAM /motor?cmd=F190  ──UART──►  Arduino  ──►  motors

    Public API:
      .go(cmd, spd, ttl)   queue a timed move
      .stop()              stop immediately
    """
    def __init__(s):
        s._base = f"http://{urllib.parse.urlparse(args.cam_url).hostname}"
        s.cmd, s.spd, s.until = 'S', 0, 0.0
        s.last = ''
        s.dist = None          # latest ultrasonic reading in cm (0 = nothing within ~2 m, None = no data)
        threading.Thread(target=s._loop, daemon=True).start()

    def _get(s, path):
        try:
            with urllib.request.urlopen(f"{s._base}/{path}", timeout=0.8) as r:
                return r.read().decode().strip()
        except Exception:
            return None

    def read_dist(s):
        """Ask the ESP32 for the Arduino's latest ultrasonic distance."""
        if args.dry_run: return None
        t = s._get("dist")
        try: v = int(t)
        except (TypeError, ValueError): return None
        if v >= 999: return None
        s.dist = v
        return v

    def go(s, cmd, spd, ttl=0.4):
        s.cmd, s.spd, s.until = cmd, spd, time.time() + ttl

    def stop(s):
        s.cmd, s.spd, s.until = 'S', 0, 0.0
        s._http("motor?cmd=S")   # send stop immediately, don’t wait for the loop

    def _http(s, path):
        try:
            urllib.request.urlopen(f"{s._base}/{path}", timeout=0.8)
        except Exception:
            pass

    def _loop(s):
        while True:
            cmd, spd = (s.cmd, s.spd) if time.time() < s.until else ('S', 0)
            text = f"{cmd}{spd}"
            if text != s.last or cmd != 'S':    # re-send move commands to feed the Arduino watchdog
                if args.dry_run:
                    if text != s.last: print(f"\n[dry-run] {text}")
                else:
                    s._http(f"motor?cmd={urllib.parse.quote(text)}")
            s.last = text
            s._n = getattr(s, '_n', 0) + 1
            if s._n % 3 == 0: s.read_dist()          # refresh the distance about every 0.45 s
            time.sleep(0.15)   # ~6 Hz: well within Arduino’s 500 ms watchdog

# ---------------- Camera link: ESP32-CAM MJPEG stream over WiFi ----------------
class Camera:
    """Reads the ESP32-CAM's MJPEG-over-HTTP stream; keeps only the newest JPEG. Auto-reconnects."""
    def __init__(s):
        s.jpg, s.t, s.online = None, 0.0, False
        s._ts: list = []            # rolling window of frame timestamps for FPS
        threading.Thread(target=s._loop, daemon=True).start()

    @property
    def fps(s) -> float:
        """Frames per second based on a 3-second rolling window. Returns 0.0 when offline."""
        now = time.monotonic()
        s._ts = [t for t in s._ts if now - t < 3.0]
        return len(s._ts) / 3.0

    def _loop(s):
        while True:
            try:
                with urllib.request.urlopen(args.cam_url, timeout=8) as r:
                    s.online = True
                    buf = b''
                    while True:
                        chunk = r.read(8192)
                        if not chunk: break
                        buf += chunk
                        while True:
                            a = buf.find(b'\xff\xd8')                  # JPEG SOI
                            if a < 0: buf = buf[-1:]; break
                            b = buf.find(b'\xff\xd9', a + 2)           # JPEG EOI
                            if b < 0: buf = buf[a:]; break
                            s.jpg, s.t = buf[a:b + 2], time.time()
                            s._ts.append(time.monotonic())
                            buf = buf[b + 2:]
                        if len(buf) > 500000: buf = b''               # safety: never grow unbounded
            except Exception as e:
                print(f"[camera] cannot connect to {args.cam_url}: {e}")
            s.online = False
            s._ts.clear()          # FPS drops to 0 immediately on disconnect
            time.sleep(1)         # retry

# ---------------- Vision thread: the ONLY place YOLO runs ----------------
class Vision:
    """Decodes + detects every new camera frame. `frame` = (timestamp, image, detections)."""
    def __init__(s):
        s.frame, s.fps = None, 0.0
        threading.Thread(target=s._loop, daemon=True).start()
    def _loop(s):
        last = 0.0
        while True:
            jpg, t = cam.jpg, cam.t
            if jpg is None or t <= last:
                time.sleep(0.01); continue
            if last: s.fps = 0.8 * s.fps + 0.2 / max(t - last, 1e-3)
            last = t
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if img is None: continue
            if args.flip: img = cv2.flip(img, -1)
            try:
                dets = detect(img); diag['err'] = ''
            except Exception as e:
                print("detect error:", e); dets = []; diag['err'] = str(e)[:90]
            diag['n'] = len(dets)
            s.frame = (t, img, dets)

class Flash:
    """Flash disabled to prevent ESP32-CAM power brownouts."""
    def __init__(s):
        s.mode = 'off'
        s.on = False
        s.level = 0.0

ard, cam = Arduino(), Camera()
vision = Vision()
flash = Flash()
stop_flag = threading.Event()
status = {'mode': 'idle', 'msg': 'Idle'}

def set_status(mode, msg):
    status['mode'], status['msg'] = mode, msg

def next_frame(after=0.0, timeout=1.0):
    """Wait for a processed frame newer than `after`. None on timeout or stop."""
    end = time.time() + timeout
    while time.time() < end and not stop_flag.is_set():
        f = vision.frame
        if f is not None and f[0] > after: return f
        time.sleep(0.01)
    return None

def wait(sec):
    end = time.time() + sec
    while time.time() < end and not stop_flag.is_set():
        time.sleep(0.02)

def burst(cmd, spd, sec):
    ard.go(cmd, spd, sec + 0.15)
    end = time.time() + sec
    while time.time() < end and not stop_flag.is_set():
        time.sleep(0.02)
    ard.stop()

def box_err(box, w):
    return (box[0] + box[2] / 2 - w / 2) / w

class Track:
    """Smooths the target's sideways offset and estimates how fast it is moving, so the rover steers
    toward where the target is GOING to be, and knows which way to turn if it loses sight of it."""
    LEAD = 0.35
    def __init__(s): s.reset()
    def reset(s): s.err, s.vel, s.t = None, 0.0, 0.0
    def update(s, err, t):
        if s.err is None:
            s.err, s.vel = err, 0.0
        else:
            dt = max(t - s.t, 1e-3)
            v = max(-1.5, min(1.5, (err - s.err) / dt))
            s.vel = 0.6 * s.vel + 0.4 * v
            s.err = 0.5 * s.err + 0.5 * err
        s.t = t
        return s.err + s.vel * s.LEAD
    def side(s, lead=0.5):
        """'L' or 'R' = which way the target most likely went; None if it was near the centre."""
        if s.err is None: return None
        p = s.err + s.vel * lead
        if abs(p) < 0.15: return None
        return 'R' if p > 0 else 'L'

def steer(box, w, h, pred=None):
    """'L', 'R', 'F' or 'S' (arrived). `pred` = predicted sideways offset from Track."""
    err = box_err(box, w) if pred is None else pred
    if box[3] / h > CLOSE_FRAC: return 'S'
    if err > ALIGN: return 'R'
    if err < -ALIGN: return 'L'
    return 'F'

def drive_for(act, near=False):
    if act == 'R': ard.go('R', args.turn, 0.6)
    elif act == 'L': ard.go('L', args.turn, 0.6)
    elif act == 'F': ard.go('F', args.slow if near else args.speed, 0.6)
    else: ard.stop()

def is_near(box, h):
    """True once the target fills over ~70% of the stop size: slow down for a precise final approach."""
    return box[3] / h > 0.70 * CLOSE_FRAC

# ---------------- Behaviours ----------------
LOST_HOLD = 1.5        # seconds to keep turning toward the predicted side after the target disappears
STOP_CM_PI = 35        # the Pi also ends a forward drive when the sonar reads below this
CLEAR_MAX = 200        # cm: readings are capped here ("wide open")
MIN_ADVANCE, MAX_ADVANCE = 30, 150
pose = {'heading': 0.0}    # estimated direction the rover faces, degrees, 0 = where the search began, + = left
tried = []                 # world directions we already drove toward (so we do not repeat ourselves)
seen_info = {'side': None, 't': 0.0}

def follow(label='person'):
    print(f"Following the {label}.")
    set_status('following', f'Following {label}')
    trk = Track()
    last, last_seen, last_act = 0.0, 0.0, 'S'
    while not stop_flag.is_set():
        f = next_frame(last, 1.0)
        if f is None:
            if time.time() - last_seen > LOST_HOLD: ard.stop()
            continue
        last, img, dets = f
        h, w = img.shape[:2]
        box = pick(dets, label)
        if box is None:
            side = trk.side() if last_seen and time.time() - last_seen < LOST_HOLD else None
            if side:
                drive_for(side)
                set_status('following', f'Lost {label}, it went {side}: turning')
            else:
                ard.stop(); last_act = 'S'
                if time.time() - last_seen > LOST_HOLD: trk.reset()
                set_status('following', f'{label} not visible')
            continue
        last_seen = time.time()
        pred = trk.update(box_err(box, w), last)
        act = steer(box, w, h, pred)
        last_act = act
        drive_for(act, is_near(box, h))
        set_status('following', f'Following {label} ({act})')
    ard.stop()

def look():
    """Stop, let the picture settle, then return a fresh processed frame."""
    wait(0.4)
    return next_frame(time.time(), 2.5)

def approach(label):
    """Drive toward the label. True = arrived, False = lost it."""
    trk = Track()
    last, lost_since, hits = 0.0, None, 0
    while not stop_flag.is_set():
        f = next_frame(last, 1.0)
        if f is None: continue
        last, img, dets = f
        h, w = img.shape[:2]
        box = pick(dets, label)
        if box is None:
            hits = 0
            lost_since = lost_since or time.time()
            lost_for = time.time() - lost_since
            side = trk.side() if lost_for < LOST_HOLD else None
            if side:
                drive_for(side)
                set_status('approaching', f'Lost {label}, it went {side}: turning')
            else:
                ard.stop()
            if lost_for > 2.5:
                seen_info['side'], seen_info['t'] = trk.side(1.0), time.time()
                print("\nLost it, searching again"); return False
            continue
        lost_since = None
        pred = trk.update(box_err(box, w), last)
        act = steer(box, w, h, pred)
        if act == 'S':
            ard.stop(); hits += 1
            if hits >= 3: return True
            continue
        hits = 0
        drive_for(act, is_near(box, h))
        set_status('approaching', f'Approaching {label} ({act})')
    ard.stop(); return False

def chase(label):
    """Target spotted: go to it. True when it has been reached."""
    set_status('approaching', f'Seen {label}, approaching')
    if approach(label):
        set_status('found', f'FOUND the {label}!')
        print(f"\nFound the {label}!")
        return True
    if not stop_flag.is_set():
        set_status('searching', f'Lost {label}, searching again...')
    return False

def turn_deg(deg):
    """Pivot about `deg` degrees (+ = left). Timed, so it is approximate."""
    if abs(deg) < 3 or stop_flag.is_set(): return
    burst('L' if deg > 0 else 'R', args.turn, abs(deg) / max(args.turn_rate, 1.0))
    pose['heading'] = (pose['heading'] + deg) % 360

def clearance():
    d = ard.read_dist()
    if d is None: return None
    return CLEAR_MAX if d == 0 else min(d, CLEAR_MAX)

def sweep(label, direction):
    """Look all the way round in steps. Returns (target_seen, {offset_degrees: clear_cm})."""
    step = max(10.0, args.scan_step)
    readings = {}
    off = 0.0
    for _ in range(int(round(360 / step))):
        if stop_flag.is_set(): return False, readings
        f = look()
        if f and pick(f[2], label) is not None: return True, readings
        c = clearance()
        readings[round(off)] = 100 if c is None else c
        set_status('searching', f'Scanning for {label}... {int(off)} deg, {readings[round(off)]} cm clear')
        turn_deg(direction * step); off += direction * step
    return False, readings

def choose_heading(readings, h0):
    """Pick the most open direction, avoiding directions we already tried. Returns (world_heading, clear_cm)."""
    offs = sorted(readings)
    best, best_score = None, -1e9
    for i, off in enumerate(offs):
        prev_c = readings[offs[i - 1]]
        next_c = readings[offs[(i + 1) % len(offs)]]
        c = readings[off]
        smooth = (prev_c + 2 * c + next_c) / 4.0           # a real opening, not one stray reading
        world = (h0 + off) % 360
        score = smooth
        for t in tried[-4:]:
            d = abs((world - t + 180) % 360 - 180)
            if d < 45: score -= 80 * (1 - d / 45)
        if score > best_score: best, best_score = (world, c), score
    return best

def advance(label, cm):
    """Drive forward about `cm`. Stops early at an obstacle. True if the target shows up on the way."""
    end = time.time() + cm / max(args.drive_cm_s, 1.0)
    while time.time() < end and not stop_flag.is_set():
        ard.go('F', args.speed, 0.4)
        d = ard.dist
        if d is not None and 0 < d < STOP_CM_PI: break
        f = vision.frame
        if f and time.time() - f[0] < 0.5 and pick(f[2], label) is not None:
            ard.stop(); return True
        time.sleep(0.05)
    ard.stop()
    return False

def find(label):
    """Search like a person would: look around, remember where we have been, head for open space."""
    print(f"Searching for: {label}")
    tried.clear()
    set_status('searching', f'Searching for {label}...')
    while not stop_flag.is_set():
        recent = seen_info['side'] and time.time() - seen_info['t'] < 10
        direction = -1 if (recent and seen_info['side'] == 'R') else 1
        h0 = pose['heading']
        seen, readings = sweep(label, direction)
        if stop_flag.is_set(): break
        if seen:
            if chase(label): return
            continue
        if not readings: continue
        world, c = choose_heading(readings, h0)
        tried.append(world)
        delta = (world - pose['heading'] + 180) % 360 - 180
        set_status('searching', f'Nothing here: heading {int(world)} deg ({c} cm clear)')
        turn_deg(delta)
        if c < 50:
            burst('B', args.slow, 0.8)               # boxed in: back out a little, then look again
            continue
        cm = max(MIN_ADVANCE, min(MAX_ADVANCE, c - 40))
        if advance(label, cm):
            if chase(label): return
    ard.stop()

# ---------------- Task control (shared by terminal and web UI) ----------------
_task, _task_lock = None, threading.Lock()

def _guard(fn, *a):
    """Run a behaviour; if it crashes, show the error on the dashboard and in the terminal."""
    try:
        fn(*a)
    except Exception as e:
        import traceback; traceback.print_exc()
        ard.stop(); set_status('idle', f'Task crashed: {e}'[:90])

def run_task(fn, *a):
    """Stop whatever is running, wait for it to exit, then start fn(*a)."""
    global _task
    with _task_lock:
        stop_flag.set()
        if _task and _task.is_alive(): _task.join(timeout=4)
        ard.stop()
        stop_flag.clear()
        _task = threading.Thread(target=_guard, args=(fn,) + a, daemon=True)
        _task.start()

def stop_task():
    stop_flag.set(); ard.stop()
    set_status('idle', 'Stopped')

def wait_for_camera():
    print("Waiting for the camera...")
    t0 = time.time()
    while cam.jpg is None and time.time() - t0 < 10: time.sleep(0.1)
    print("Camera OK" if cam.jpg else f"WARNING: no frames from {args.cam_url} (check ESP32 IP / WiFi)")

# ---------------- Terminal mode: python3 rover_ai.py ----------------
def main():
    cmds = queue.Queue()
    def stdin_loop():
        for line in sys.stdin:
            cmds.put(line.strip().lower())
    threading.Thread(target=stdin_loop, daemon=True).start()
    wait_for_camera()
    print("Commands: follow | find <object> | stop | quit")
    if args.dry_run: print("DRY RUN: nothing is sent to the motors")
    try:
        while True:
            c = cmds.get()
            if c in ('follow', 'follow me'):
                run_task(follow, 'person')
            elif c.split(' ')[0] in ('find', 'search') and ' ' in c:
                label = norm(c.split(' ', 1)[1].replace('look for ', ''))
                if label not in NAMES: print(f"I don't know '{label}'. Known: {', '.join(NAMES)}")
                else: run_task(find, label)
            elif c == 'stop':
                stop_task()
            elif c == 'quit':
                break
    finally:
        stop_task(); time.sleep(0.4)

if __name__ == '__main__':
    main()
