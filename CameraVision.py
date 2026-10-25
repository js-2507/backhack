import cv2
import mediapipe as mp
import numpy as np
import time
import winsound
import urllib.request
import tempfile
import wave
from pathlib import Path

from mediapipe.tasks import python
from mediapipe.tasks.python import vision


# ---------- Settings ----------
CAMERA_INDEX = 0
ANGLE_THRESHOLD = 10
ALERT_DELAY = 3
ALERT_COOLDOWN = 10
CALIBRATION_FRAMES = 30

BEEP_COUNT = 10
BEEP_FREQUENCY = 1000
BEEP_DURATION = 200       # Milliseconds
BEEP_PAUSE = 0.15         # Seconds

MODEL_PATH = Path(__file__).resolve().parent / "pose_landmarker_lite.task"
MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/"
    "pose_landmarker/pose_landmarker_lite/float16/1/"
    "pose_landmarker_lite.task"
)


# ---------- Download the model once ----------
if not MODEL_PATH.exists():
    print("Downloading pose detection model...")
    temporary_path = MODEL_PATH.with_suffix(".download")

    try:
        with urllib.request.urlopen(MODEL_URL, timeout=60) as response:
            temporary_path.write_bytes(response.read())

        temporary_path.replace(MODEL_PATH)
        print("Model downloaded.")
    finally:
        temporary_path.unlink(missing_ok=True)


# ---------- Cancellable alert sound ----------
class PostureAlert:
    def __init__(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / "posture_alert.wav"
        self.active = False
        self.ends_at = 0.0

        sample_rate = 44100
        beep_samples = int(sample_rate * BEEP_DURATION / 1000)
        pause_samples = int(sample_rate * BEEP_PAUSE)

        t = np.arange(beep_samples) / sample_rate
        tone = 0.35 * np.sin(2 * np.pi * BEEP_FREQUENCY * t)

        # Fade edges slightly to avoid clicks.
        fade_samples = min(int(sample_rate * 0.005), beep_samples // 2)

        if fade_samples:
            tone[:fade_samples] *= np.linspace(0, 1, fade_samples)
            tone[-fade_samples:] *= np.linspace(1, 0, fade_samples)

        parts = []

        for i in range(BEEP_COUNT):
            parts.append(tone)

            if i < BEEP_COUNT - 1:
                parts.append(np.zeros(pause_samples))

        audio = np.concatenate(parts)
        pcm = (audio * 32767).astype("<i2")

        self.duration = len(audio) / sample_rate

        with wave.open(str(self.path), "wb") as output:
            output.setnchannels(1)
            output.setsampwidth(2)
            output.setframerate(sample_rate)
            output.writeframes(pcm.tobytes())

    def is_playing(self):
        if self.active and time.monotonic() >= self.ends_at:
            self.active = False

        return self.active

    def start(self):
        if self.is_playing():
            return

        try:
            winsound.PlaySound(
                str(self.path),
                winsound.SND_FILENAME
                | winsound.SND_ASYNC
                | winsound.SND_NODEFAULT,
            )

            self.active = True
            self.ends_at = time.monotonic() + self.duration

        except RuntimeError as error:
            print(f"Could not play alert sound: {error}")

    def stop(self):
        if self.active:
            # Stop the current sound immediately.
            winsound.PlaySound(None, 0)
            self.active = False

    def close(self):
        self.stop()
        self.directory.cleanup()


# ---------- Helper functions ----------
def calculate_angle(a, b, c):
    """Calculate angle ABC, with B as the vertex."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    c = np.asarray(c, dtype=float)

    ba = a - b
    bc = c - b

    denominator = np.linalg.norm(ba) * np.linalg.norm(bc)

    if denominator < 0.000001:
        return None

    cosine = np.clip(np.dot(ba, bc) / denominator, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def draw_angle(frame, a, b, c, angle, color):
    cv2.line(frame, a, b, color, 2)
    cv2.line(frame, b, c, color, 2)

    cv2.putText(
        frame,
        f"{angle:.1f}",
        (b[0] + 10, b[1] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        color,
        2,
    )


def visible(landmark):
    return (
        landmark.visibility >= 0.6
        and landmark.presence >= 0.6
        and 0 <= landmark.x <= 1
        and 0 <= landmark.y <= 1
    )


def pixel(landmark, width, height):
    return (
        int(landmark.x * width),
        int(landmark.y * height),
    )


CONNECTIONS = [
    (7, 11),
    (8, 12),
    (11, 12),
    (11, 13),
    (13, 15),
    (12, 14),
    (14, 16),
    (11, 23),
    (12, 24),
    (23, 24),
]


def draw_skeleton(frame, landmarks, width, height):
    for start, end in CONNECTIONS:
        if visible(landmarks[start]) and visible(landmarks[end]):
            a = pixel(landmarks[start], width, height)
            b = pixel(landmarks[end], width, height)

            cv2.line(frame, a, b, (0, 200, 0), 2)
            cv2.circle(frame, a, 4, (255, 255, 255), -1)
            cv2.circle(frame, b, 4, (255, 255, 255), -1)


# ---------- Calibration and alert variables ----------
is_calibrated = False
calibration_shoulder_angles = []
calibration_neck_angles = []

baseline_shoulder = 0.0
baseline_neck = 0.0

filtered_shoulder = None
filtered_neck = None

poor_posture_since = None
last_alert_time = float("-inf")
last_valid_time = None
ear_index = None


# ---------- Initialize MediaPipe ----------
options = vision.PoseLandmarkerOptions(
    base_options=python.BaseOptions(
        model_asset_path=str(MODEL_PATH)
    ),
    running_mode=vision.RunningMode.VIDEO,
    num_poses=1,
    min_pose_detection_confidence=0.5,
    min_pose_presence_confidence=0.5,
    min_tracking_confidence=0.5,
)

cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    cap.release()
    raise RuntimeError(
        "Could not open camera. Try CAMERA_INDEX = 1 or 2 "
        "and close other apps using the camera."
    )

alert = None

print("Sit upright while calibration runs.")
print("Press C to recalibrate. Press Q to quit.")

started = time.monotonic()
last_timestamp_ms = -1

try:
    alert = PostureAlert()

    with vision.PoseLandmarker.create_from_options(options) as pose:
        while cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                print("Camera disconnected or frame could not be read.")
                break

            now = time.monotonic()
            height, width = frame.shape[:2]

            timestamp_ms = max(
                last_timestamp_ms + 1,
                int((now - started) * 1000),
            )
            last_timestamp_ms = timestamp_ms

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=rgb_frame,
            )

            results = pose.detect_for_video(mp_image, timestamp_ms)

            status = "No reliable pose detected"
            color = (0, 255, 255)
            valid_measurement = False
            detail_lines = []

            if results.pose_landmarks:
                landmarks = results.pose_landmarks[0]
                draw_skeleton(frame, landmarks, width, height)

                candidate_ear = ear_index

                if candidate_ear is None:
                    candidate_ear = max(
                        (7, 8),
                        key=lambda i: landmarks[i].visibility,
                    )

                shoulder_index = 11 if candidate_ear == 7 else 12
                required = (11, 12, candidate_ear)

                if all(visible(landmarks[i]) for i in required):
                    left_shoulder = pixel(landmarks[11], width, height)
                    right_shoulder = pixel(landmarks[12], width, height)
                    ear = pixel(landmarks[candidate_ear], width, height)

                    neck_shoulder = (
                        left_shoulder
                        if shoulder_index == 11
                        else right_shoulder
                    )

                    shoulder_reference = (
                        right_shoulder[0],
                        right_shoulder[1] - 100,
                    )
                    neck_reference = (
                        neck_shoulder[0],
                        neck_shoulder[1] - 100,
                    )

                    shoulder_angle = calculate_angle(
                        left_shoulder,
                        right_shoulder,
                        shoulder_reference,
                    )
                    neck_angle = calculate_angle(
                        ear,
                        neck_shoulder,
                        neck_reference,
                    )

                    if shoulder_angle is not None and neck_angle is not None:
                        valid_measurement = True

                        if (
                            last_valid_time is None
                            or now - last_valid_time > 0.75
                        ):
                            alert.stop()
                            filtered_shoulder = shoulder_angle
                            filtered_neck = neck_angle
                            poor_posture_since = None
                        else:
                            dt = now - last_valid_time
                            alpha = 1 - np.exp(-dt / 0.3)

                            filtered_shoulder += alpha * (
                                shoulder_angle - filtered_shoulder
                            )
                            filtered_neck += alpha * (
                                neck_angle - filtered_neck
                            )

                        last_valid_time = now

                        draw_angle(
                            frame,
                            left_shoulder,
                            right_shoulder,
                            shoulder_reference,
                            shoulder_angle,
                            (255, 150, 0),
                        )
                        draw_angle(
                            frame,
                            ear,
                            neck_shoulder,
                            neck_reference,
                            neck_angle,
                            (0, 255, 255),
                        )

                        if not is_calibrated:
                            alert.stop()
                            ear_index = candidate_ear

                            calibration_shoulder_angles.append(
                                shoulder_angle
                            )
                            calibration_neck_angles.append(neck_angle)

                            count = len(calibration_neck_angles)
                            status = (
                                f"Sit upright: calibrating "
                                f"{count}/{CALIBRATION_FRAMES}"
                            )

                            if count >= CALIBRATION_FRAMES:
                                baseline_shoulder = float(
                                    np.mean(calibration_shoulder_angles)
                                )
                                baseline_neck = float(
                                    np.mean(calibration_neck_angles)
                                )

                                is_calibrated = True
                                filtered_shoulder = baseline_shoulder
                                filtered_neck = baseline_neck
                                poor_posture_since = None

                                print("Calibration complete.")

                        else:
                            shoulder_change = abs(
                                filtered_shoulder - baseline_shoulder
                            )
                            neck_change = abs(
                                filtered_neck - baseline_neck
                            )

                            changed = (
                                shoulder_change > ANGLE_THRESHOLD
                                or neck_change > ANGLE_THRESHOLD
                            )

                            if changed:
                                if poor_posture_since is None:
                                    poor_posture_since = now

                                duration = now - poor_posture_since

                                if duration >= ALERT_DELAY:
                                    status = "Posture reminder"
                                    color = (0, 0, 255)

                                    if (
                                        now - last_alert_time
                                        >= ALERT_COOLDOWN
                                        and not alert.is_playing()
                                    ):
                                        print(
                                            "Sustained posture change detected."
                                        )
                                        last_alert_time = now
                                        alert.start()
                                else:
                                    # Do not continue an old alert while
                                    # waiting for a new sustained change.
                                    alert.stop()
                                    status = (
                                        f"Posture changed: "
                                        f"{duration:.1f}/{ALERT_DELAY}s"
                                    )
                                    color = (0, 165, 255)

                            else:
                                # Cancel all remaining beeps, including
                                # the sound currently playing.
                                alert.stop()

                                poor_posture_since = None
                                status = "Within calibrated range"
                                color = (0, 255, 0)

                            detail_lines = [
                                f"Shoulder change: {shoulder_change:.1f} deg",
                                f"Head-line change: {neck_change:.1f} deg",
                                f"Trigger threshold: {ANGLE_THRESHOLD} deg",
                            ]

            if not valid_measurement:
                alert.stop()
                poor_posture_since = None
                last_valid_time = None

                if not is_calibrated:
                    calibration_shoulder_angles.clear()
                    calibration_neck_angles.clear()
                    ear_index = None

            cv2.putText(
                frame,
                status,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv2.LINE_AA,
            )

            for i, line in enumerate(detail_lines):
                cv2.putText(
                    frame,
                    line,
                    (10, 60 + i * 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            cv2.putText(
                frame,
                "C: recalibrate | Q: quit",
                (10, height - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            cv2.imshow("Posture Corrector", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            if key == ord("c"):
                alert.stop()

                is_calibrated = False
                calibration_shoulder_angles.clear()
                calibration_neck_angles.clear()
                filtered_shoulder = None
                filtered_neck = None
                poor_posture_since = None
                last_valid_time = None
                last_alert_time = float("-inf")
                ear_index = None

                print("Recalibrating. Sit upright.")

finally:
    if alert is not None:
        alert.close()

    cap.release()
    cv2.destroyAllWindows()