import os
import cv2
import time
import queue
import threading
import numpy as np
from typing import Callable, Optional, Any, List, Dict, Tuple

from ultralytics import YOLO
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V


# ─────────────────────────────────────────────
#  Per-instance barrel tracker (world-space)
# ─────────────────────────────────────────────

class BarrelTracker:
    """
    Tracks individual barrel instances using the drone's world position from Gazebo.

    When a barrel is detected, we record WHERE THE DRONE WAS in world coordinates
    (x, y, z metres). A future detection of the same class is considered the SAME
    barrel if the drone is within `drone_dist_threshold` metres of that earlier
    position — meaning it is looking at the same part of the maze.

    Each unique barrel instance gets its own save counter capped at `max_saves`.
    """

    def __init__(self, max_saves: int = 5, drone_dist_threshold: float = 3.0):
        """
        Args:
            max_saves:              Max images saved per individual barrel.
            drone_dist_threshold:   If the drone is within this many metres of
                                    where it previously detected a barrel of the
                                    same class, assume it's the same barrel.
                                    Tune based on how spread out the barrels are
                                    in the maze (default 3 m is a reasonable start).
        """
        self.max_saves = max_saves
        self.drone_dist_threshold = drone_dist_threshold
        self._lock = threading.Lock()
        # List of {"class": str, "dx": float, "dy": float, "dz": float,
        #          "count": int, "id": int}
        # dx/dy/dz = drone world position at first detection of this barrel
        self._barrels: List[Dict] = []
        self._next_id = 1

    def should_save(
        self,
        class_name: str,
        drone_pos: Dict[str, float]   # {"x": ..., "y": ..., "z": ...}
    ) -> Tuple[bool, int]:
        """
        Decide whether to save an image for this detection.

        Returns:
            (should_save, barrel_id)
        """
        dx, dy, dz = drone_pos["x"], drone_pos["y"], drone_pos["z"]

        with self._lock:
            for barrel in self._barrels:
                if barrel["class"] != class_name:
                    continue
                dist = (
                    (barrel["dx"] - dx) ** 2 +
                    (barrel["dy"] - dy) ** 2 +
                    (barrel["dz"] - dz) ** 2
                ) ** 0.5

                if dist <= self.drone_dist_threshold:
                    # Same barrel — check quota
                    if barrel["count"] < self.max_saves:
                        barrel["count"] += 1
                        return True, barrel["id"]
                    else:
                        return False, barrel["id"]   # cap reached

            # No match → new barrel
            new_barrel = {
                "class": class_name,
                "dx": dx, "dy": dy, "dz": dz,
                "count": 1,
                "id": self._next_id
            }
            self._next_id += 1
            self._barrels.append(new_barrel)
            print(
                f"[Tracker] New barrel → '{class_name}' barrel #{new_barrel['id']} "
                f"(drone at x={dx:.1f} y={dy:.1f} z={dz:.1f})"
            )
            return True, new_barrel["id"]

    def summary(self) -> None:
        with self._lock:
            print("\n[Tracker] ── Detection summary ──")
            for b in self._barrels:
                print(
                    f"  Barrel #{b['id']:02d}  class={b['class']:20s}  "
                    f"images saved={b['count']}  "
                    f"first seen at drone pos ({b['dx']:.1f}, {b['dy']:.1f}, {b['dz']:.1f})"
                )


# ─────────────────────────────────────────────
#  Detector
# ─────────────────────────────────────────────

class Detector:
    def __init__(
        self,
        model_path: str = "my_model.pt",
        confidence_threshold: float = 0.85,
        callback: Optional[Callable[[List[Dict[str, Any]], np.ndarray, Optional[Any]], None]] = None,
        num_workers: int = 1,
        device: str = "cpu",
        save_dir: str = "./detections",
        enable_display: bool = True,
        display_window_name: str = "YOLO Detections",
        max_saves_per_barrel: int = 5,
        drone_dist_threshold: float = 3.0
    ):
        self.model = YOLO(model_path).to(device)
        self.conf_threshold = confidence_threshold
        self.callback = callback
        # maxsize=1: if a worker is busy, the next incoming frame overwrites the
        # waiting one instead of queuing behind it — keeps inference on the latest frame
        self.queue = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.workers = []

        self.save_dir = os.path.abspath(save_dir)
        os.makedirs(self.save_dir, exist_ok=True)
        self._file_counter = 0
        self._counter_lock = threading.Lock()

        self.tracker = BarrelTracker(
            max_saves=max_saves_per_barrel,
            drone_dist_threshold=drone_dist_threshold
        )

        self.enable_display = enable_display
        self.display_window_name = display_window_name
        self.display_queue = queue.Queue(maxsize=1)
        self.display_thread = None

        self._start_workers(num_workers)

        if self.enable_display:
            self.display_thread = threading.Thread(target=self._display_worker, daemon=True)
            self.display_thread.start()

    def _start_workers(self, num_workers: int) -> None:
        for _ in range(num_workers):
            t = threading.Thread(target=self._worker, daemon=False)
            t.start()
            self.workers.append(t)

    def submit_image(self, image: np.ndarray, context: Optional[Dict[str, Any]] = None) -> None:
        if context is None:
            context = {}
        try:
            self.queue.put_nowait((image, context))
        except queue.Full:
            pass  # worker still busy — drop stale frame, keep moving

    def _get_next_filename(self, class_name: str, barrel_id: int) -> str:
        with self._counter_lock:
            self._file_counter += 1
            ts = int(time.time() * 1000)
            safe_name = class_name.replace(" ", "_")
            return f"{safe_name}_barrel{barrel_id:02d}_{self._file_counter}_{ts}.jpg"

    def _display_worker(self) -> None:
        cv2.namedWindow(self.display_window_name, cv2.WINDOW_AUTOSIZE)
        while not self.stop_event.is_set():
            try:
                img = self.display_queue.get(timeout=0.05)
                if img is not None:
                    cv2.imshow(self.display_window_name, img)
                    cv2.waitKey(1)
            except queue.Empty:
                continue
            except cv2.error as e:
                print(f"[Display] CV2 error: {e}")
                break
        cv2.destroyWindow(self.display_window_name)

    def _worker(self) -> None:
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                image, context = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                results = self.model(image, verbose=False, conf=self.conf_threshold)

                detections = []
                annotated_image = None
                has_detections = False

                for result in results:
                    boxes = result.boxes
                    if boxes is not None and len(boxes) > 0:
                        has_detections = True
                        for box in boxes:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                            conf = float(box.conf[0].cpu().item())
                            cls_id = int(box.cls[0].cpu().item())
                            detections.append({
                                "bbox": [x1, y1, x2, y2],
                                "confidence": conf,
                                "class_id": cls_id,
                                "class_name": self.model.names[cls_id]
                            })
                        annotated_image = result.plot()

                if has_detections and annotated_image is not None:
                    saved_paths = []
                    drone_pos = context.get("drone_pos", {"x": 0.0, "y": 0.0, "z": 0.0})

                    for det in detections:
                        save, barrel_id = self.tracker.should_save(
                            det["class_name"], drone_pos
                        )
                        det["barrel_id"] = barrel_id
                        if save:
                            filename = self._get_next_filename(det["class_name"], barrel_id)
                            filepath = os.path.join(self.save_dir, filename)
                            cv2.imwrite(filepath, annotated_image)
                            saved_paths.append(filepath)

                    context["saved_paths"] = saved_paths

                    if self.enable_display:
                        try:
                            self.display_queue.put_nowait(annotated_image)
                        except queue.Full:
                            pass

                    if self.callback:
                        try:
                            self.callback(detections, annotated_image, context)
                        except Exception as e:
                            print(f"[Detector] Callback error: {e}")

            except Exception as e:
                print(f"[Detector] Inference error: {e}")
            finally:
                self.queue.task_done()
                del image, context, results, detections, annotated_image

    def stop(self) -> None:
        self.stop_event.set()
        for t in self.workers:
            t.join()
        if self.display_thread and self.display_thread.is_alive():
            self.display_thread.join()
        self.tracker.summary()
        print("[Detector] All workers and display thread stopped.")

    def set_display(self, enabled: bool) -> None:
        self.enable_display = enabled


# ─────────────────────────────────────────────
#  Gazebo topics
# ─────────────────────────────────────────────

IMAGE_TOPIC = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"
POSE_TOPIC  = "/world/roboverse/dynamic_pose/info"   # broadcasts ALL model poses each tick
DRONE_NAME  = "x500_vision_0"

_detector: Optional[Detector] = None

# Shared drone position updated by the pose subscriber
_drone_pos: Dict[str, float] = {"x": 0.0, "y": 0.0, "z": 0.0}
_drone_pos_lock = threading.Lock()


# ─────────────────────────────────────────────
#  Gazebo callbacks
# ─────────────────────────────────────────────

def pose_callback(msg: Pose_V) -> None:
    """Updates the drone world position from Gazebo's dynamic pose broadcast."""
    global _drone_pos
    for pose in msg.pose:
        if pose.name == DRONE_NAME:
            with _drone_pos_lock:
                _drone_pos = {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                }
            break


def image_callback(msg: Image) -> None:
    """Converts the Gazebo image and submits it to the detector with current drone pos."""
    if _detector is None:
        return

    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    with _drone_pos_lock:
        pos_snapshot = dict(_drone_pos)   # copy so the worker sees a consistent value

    _detector.submit_image(
        frame_bgr.copy(),
        context={
            "timestamp": msg.header.stamp.sec,
            "drone_pos": pos_snapshot        # world position at the moment of this frame
        }
    )


def detection_callback(detections: List[Dict[str, Any]], annotated_image: np.ndarray, context: Any) -> None:
    saved = context.get("saved_paths", [])
    pos = context.get("drone_pos", {})
    print(
        f"[Callback] {len(detections)} detection(s)  "
        f"drone=({pos.get('x',0):.1f}, {pos.get('y',0):.1f}, {pos.get('z',0):.1f})  "
        f"{len(saved)} saved"
    )
    for d in detections:
        print(f"  · barrel #{d.get('barrel_id','?'):02d}  {d['class_name']:20s}  conf={d['confidence']:.2f}")


# ─────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────

def main():
    global _detector

    _detector = Detector(
        model_path="my_model.pt",
        confidence_threshold=0.8,
        callback=detection_callback,
        num_workers=2,
        device="cpu",
        save_dir=os.path.join(os.path.dirname(os.path.abspath(__file__)), "detections"),
        enable_display=True,
        display_window_name="x500_vision_0 — YOLO",
        max_saves_per_barrel=5,
        drone_dist_threshold=3.0    # metres — increase if barrels are far apart in the maze
    )

    node = Node()

    # Subscribe to drone pose
    if node.subscribe(Pose_V, POSE_TOPIC, pose_callback):
        print(f"[GZ] Subscribed to pose topic: {POSE_TOPIC}")
    else:
        print(f"[GZ] WARNING: Could not subscribe to {POSE_TOPIC} — drone position will default to (0,0,0)")

    # Subscribe to camera
    if node.subscribe(Image, IMAGE_TOPIC, image_callback):
        print(f"[GZ] Subscribed to camera topic: {IMAGE_TOPIC}")
        print("[GZ] Press Ctrl+C to stop.")
    else:
        print(f"[GZ] Failed to subscribe to {IMAGE_TOPIC}. Is Gazebo running?")
        _detector.stop()
        return

    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[Main] Shutting down …")
    finally:
        _detector.stop()
        cv2.destroyAllWindows()
        print("[Main] Clean exit.")


if __name__ == "__main__":
    main()