from flask import Flask, Response, render_template, request, jsonify
import time
import cv2

import rover_ai as rover          # starts camera, Arduino link and vision thread

app = Flask(__name__)

def generate_frames():
    """Streams the latest processed frame, or directly from camera if vision processing hasn't started."""
    last_t = 0.0
    while True:
        f = rover.vision.frame
        if f is not None and f[0] > last_t:
            last_t, img, dets = f
            img = img.copy()
            for label, conf, box in dets:
                x, y, bw, bh = (int(v) for v in box)
                cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                cv2.putText(img, f"{label} {conf:.2f}", (x, max(20, y - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
        elif (f is None or time.time() - f[0] > 1.5) and rover.cam.jpg is not None and rover.cam.t > last_t:
            # Fallback: display raw camera frames directly if vision pipeline is still initializing
            last_t = rover.cam.t
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + rover.cam.jpg + b'\r\n')
        else:
            time.sleep(0.02)

@app.route('/')
def index():
    return render_template('index.html', names=rover.NAMES)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/command', methods=['POST'])
def command():
    data = request.get_json(silent=True) or {}
    c = data.get('command', '').strip().lower()

    if c in ('follow', 'follow me'):
        rover.run_task(rover.follow, 'person')
    elif c.split(' ')[0] in ('find', 'search') and ' ' in c:
        label = rover.norm(c.split(' ', 1)[1])
        if label not in rover.NAMES:
            return jsonify({"status": "error", "message": f"I don't know '{label}'"}), 400
        rover.run_task(rover.find, label)
    elif c == 'stop':
        rover.stop_task()
    else:
        return jsonify({"status": "error", "message": "unknown command"}), 400
    return jsonify({"status": "success", "command": c})

@app.route('/status')
def status():
    return jsonify({"cam": rover.cam.online, "cam_fps": round(rover.cam.fps, 1), "ai_fps": round(rover.vision.fps, 1),
                    "mode": rover.status['mode'], "msg": rover.status['msg'], "diag": rover.diag, "dist": rover.ard.dist})

if __name__ == '__main__':
    rover.wait_for_camera()
    # debug/reloader must stay off: a second process would try to open the serial ports again
    app.run(host='0.0.0.0', port=5000, threaded=True, debug=False, use_reloader=False)
