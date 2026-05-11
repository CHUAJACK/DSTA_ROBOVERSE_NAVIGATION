import time
import numpy as np
import cv2
import os
from ultralytics import YOLO
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
import asyncio
import queue

class GZPhotoDetectorSaver:
    def __init__(self, topic, save_dir="output", model_path="yolov8n.pt", burst_size=20, threshold=0.5):
        self.topic = topic
        self.save_dir = save_dir
        self.burst_size = burst_size
        self.threshold = threshold  # Confidence threshold for saving

        self.img_queue = queue.LifoQueue(maxsize=50)

        
        if os.path.exists(model_path):
            print(f"Loading model: {model_path} (Threshold: {self.threshold})")
            self.model = YOLO(model_path)
        else:
            print(f"WARNING: Model file '{model_path}' not found. Detection disabled.")
            self.model = None
        
        self.is_detecting = False
        self.is_saving = False
        self.frames_remaining = 0
        self.show = False
        
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

    def trigger_detection_burst(self,numofframes=30):
        if self.model:
            self.burst_size = numofframes
            self.is_detecting = True
            self.is_saving = False
            print("Triggered Camera Detection Task")

    def trigger_capture_burst(self,numofframes=30):
        self.burst_size = numofframes
        self.is_detecting = False
        self.is_saving = True
        print("Triggered Camera Capture Task")


    def _image_callback(self, msg: Image):
        try:
            self.img_queue.put(msg)
        except queue.Full:
            self.image_queue.queue.clear()
            self.img_queue.put(msg)


    async def _worker(self):
        """The async background consumer."""
        print("Camera Background worker started.")
        while True:
            img = self.img_queue.get_nowait()
            await self.loop.run_in_executor(None, self._process_task, img)
            self.queue.task_done()

    def _process_task(self, img):
        """Blocking logic: YOLO inference and Disk I/O."""
        frame = np.frombuffer(img.data, dtype=np.uint8)
        frame = frame.reshape((msg.height, msg.width, 3))
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        displayframe = None
        if self.frames_remaining > 0 and (self.is_detecting or self.is_saving):
            if self.is_detecting and self.model:
                results = self.model(frame, conf=self.threshold, verbose=False)
                if len(results.boxes) > 0:
                    annotated = results.plot()
                    path = os.path.join(self.save_dir, f"det_{task['ts']}.jpg")
                    cv2.imwrite(path, annotated)
                    self.show = True
                    displayframe = annotated

            elif self.is_saving:
                path = os.path.join(self.save_dir, f"raw_{task['ts']}.jpg")
                cv2.imwrite(path, frame)
            self.frames_remaining = self.frames_remaining - 1
        else:
            self.is_saving = False
            self.is_detecting = False           
            print("Camera Task Complete")

        if self.show:
            cv2.imshow("Gazebo Photo Booth", displayframe)
            cv2.waitKey(1)
            self.show = False

    async def run(self):
        """Entry point to start the subscription and worker."""
        self.loop = asyncio.get_running_loop()
        self.node = Node()

        if self.node.subscribe(Image, self.topic, self._image_callback):
            print(f"Subscribed to {self.topic}. No rendering (Headless).")
            worker_task = asyncio.create_task(self._worker())
            await asyncio.Future()  # Run forever
        else:
            print("Failed to subscribe.")
   
async def main():
    TOPIC = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"
    detector = GZPhotoDetectorSaver(topic=TOPIC,save_dir="output", model_path="yolov8n.pt", burst_size=20, threshold=0.5)
    # Run the detector
    await detector.run()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down...")