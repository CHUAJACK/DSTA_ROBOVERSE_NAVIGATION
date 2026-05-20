import os
import cv2
import time
import queue
import threading
import numpy as np
from typing import Callable, Optional, Any, List, Dict

from ultralytics import YOLO
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


# ─────────────────────────────────────────────
#  Detector (unchanged logic from Detector.py)
# ─────────────────────────────────────────────

class Detector:
    def __init__(
        self,
        model_path: str = "my_model.pt",
        confidence_threshold: float = 0.85,
        callback: Optional[Callable[[List[Dict[str, Any]], np.ndarray, Optional[Any]], None]] = None,
        num_workers: int = 1,
        device: str = "cpu",
        save_dir: str = "./detected_images",
        enable_display: bool = True,
        display_window_name: str = "YOLO Detections"
    ):
        self.model = YOLO(model_path).to(device)
        self.conf_threshold = confidence_threshold
        self.callback = callback
        self.queue = queue.Queue()
        self.stop_event = threading.Event()
        self.workers = []

        self.save_dir = os.path.abspath(save_dir)
        os.makedirs(self.save_dir, exist_ok=True)
        self._file_counter = 0
        self._counter_lock = threading.Lock()

        self.enable_display = enable_display
        self.display_window_name = display_window_name
        # maxsize=1 → always show the latest frame, drop stale ones
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
        self.queue.put((image, context))

    def _get_next_filename(self) -> str:
        with self._counter_lock:
            self._file_counter += 1
            ts = int(time.time() * 1000)
            return f"det_{self._file_counter}_{ts}.jpg"

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
                    filename = self._get_next_filename()
                    filepath = os.path.join(self.save_dir, filename)
                    cv2.imwrite(filepath, annotated_image)
                    context["saved_path"] = filepath

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
        print("[Detector] All workers and display thread stopped.")

    def set_display(self, enabled: bool) -> None:
        self.enable_display = enabled


# ─────────────────────────────────────────────
#  Gazebo → Detector bridge
# ─────────────────────────────────────────────

IMAGE_TOPIC = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"

# Global detector reference so the callback can reach it
_detector: Optional[Detector] = None


def detection_callback(detections: List[Dict[str, Any]], annotated_image: np.ndarray, context: Any) -> None:
    """Called by the Detector every time objects are found in a frame."""
    print(f"[Callback] {len(detections)} detection(s) — saved to {context.get('saved_path', 'N/A')}")
    for d in detections:
        print(f"  · {d['class_name']:20s}  conf={d['confidence']:.2f}  bbox={[round(v) for v in d['bbox']]}")


def image_callback(msg: Image) -> None:
    """Gazebo subscriber callback — converts the raw message and feeds the Detector."""
    if _detector is None:
        return

    # Raw bytes → NumPy array
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

    # Gazebo publishes RGB; OpenCV / YOLO expects BGR
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Hand off to the async detector (non-blocking)
    _detector.submit_image(
        frame_bgr.copy(),           # copy so Gazebo can reuse its buffer
        context={"timestamp": msg.header.stamp.sec}
    )


def main():
    global _detector

    # ── Detector setup ──────────────────────────────────────────────
    _detector = Detector(
        model_path="my_model.pt",       # ← swap in your weights file
        confidence_threshold=0.85,       # lower than 0.85 is safer for live testing
        callback=detection_callback,
        num_workers=2,                  # two inference threads keeps up with 30 fps
        device="cpu",                   # change to "cuda:0" if you have a GPU
        save_dir=os.path.join(os.path.dirname(os.path.abspath(__file__)), "detections"),
        enable_display=True,
        display_window_name="x500_vision_0 — YOLO"
    )

    # ── Gazebo subscriber setup ──────────────────────────────────────
    node = Node()

    if node.subscribe(Image, IMAGE_TOPIC, image_callback):
        print(f"[GZ] Subscribed to {IMAGE_TOPIC}")
        print("[GZ] Press Ctrl+C to stop.")
    else:
        print(f"[GZ] Failed to subscribe to {IMAGE_TOPIC}. Is Gazebo running?")
        _detector.stop()
        return

    # ── Main loop ────────────────────────────────────────────────────
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