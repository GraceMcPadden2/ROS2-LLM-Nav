import time

class MockMotionController:
    def start(self):
        print("[MOCK] Controller started")
    def walk(self, duration_s=0.05, speed_mps=0.3):
        print(f"[MOCK] walk(duration_s={duration_s}, speed_mps={speed_mps})")
        time.sleep(duration_s)
    def turn_in_place(self, duration_s=0.05, angular_z=0.8):
        print(f"[MOCK] turn_in_place(duration_s={duration_s}, angular_z={angular_z})")
        time.sleep(duration_s)
    def sit(self, duration_s=0.05):
        print(f"[MOCK] sit(duration_s={duration_s})")
        time.sleep(duration_s)
    def stretch(self, duration_s=0.05):
        print(f"[MOCK] stretch(duration_s={duration_s})")
        time.sleep(duration_s)
    def stand_up(self):
        print("[MOCK] stand_up()")
    def stand_down(self):
        print("[MOCK] stand_down()")
    def recovery_stand(self):
        print("[MOCK] recovery_stand()")
    def say(self, text: str):
        print(f"[MOCK] say: {text}")

    # --- COCO Perception ---
    MOCK_SCENE = [
        {"label": "person", "score": 0.97, "bbox": [100, 80,  210, 340]},
        {"label": "chair",  "score": 0.93, "bbox": [300, 150, 450, 400]},
        {"label": "dog",    "score": 0.91, "bbox": [50,  200, 160, 370]},
        {"label": "bottle", "score": 0.88, "bbox": [520, 300, 560, 420]},
    ]

    def get_detections(self):
        """Returns mock COCO detections."""
        print("[MOCK] get_detections()")
        return self.MOCK_SCENE

    def find_object(self, label: str):
        """Returns best match for a label or None."""
        print(f"[MOCK] find_object({label})")
        matches = [d for d in self.get_detections() if d["label"].lower() == label.lower()]
        return max(matches, key=lambda d: d["score"]) if matches else None