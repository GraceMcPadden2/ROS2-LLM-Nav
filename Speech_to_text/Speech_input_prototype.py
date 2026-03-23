import whisper
import sounddevice as sd
from scipy.io.wavfile import write
import time

# Load Whisper model (first run will download)
model = whisper.load_model("base")

# ----------------------------
# Audio Recording
# ----------------------------
def record_audio(duration=4, fs=16000):
    print("\n Speak now...")
    audio = sd.rec(int(duration * fs), samplerate=fs, channels=1)
    sd.wait()
    filename = "temp.wav"
    write(filename, fs, audio)
    return filename

# ----------------------------
# Speech to Text
# ----------------------------
def transcribe_audio(file_path):
    result = model.transcribe(file_path)
    return result["text"].strip()

# ----------------------------
# Command Parsing
# ----------------------------
def parse_command(text):
    text = text.lower()

    # WALK
    if "walk" in text or "forward" in text:
        duration = extract_number(text) or 1.0
        return {
            "action": "walk",
            "duration_s": duration,
            "speed_mps": 0.3
        }

    # TURN
    elif "turn" in text:
        duration = extract_number(text) or 1.0
        direction = -0.8 if "right" in text else 0.8
        return {
            "action": "turn_in_place",
            "duration_s": duration,
            "angular_z": direction
        }

    # SIT
    elif "sit" in text:
        return {
            "action": "sit",
            "duration_s": 3.0
        }

    # STAND
    elif "stand" in text:
        return {
            "action": "stand_up"
        }

    # STRETCH
    elif "stretch" in text:
        return {
            "action": "stretch",
            "duration_s": 3.0
        }

    return {"action": "unknown"}

# ----------------------------
# Extract numbers like "2 seconds"
# ----------------------------
def extract_number(text):
    word_to_num = {
        "one": 1,
        "two": 2,
        "three": 3,
        "four": 4,
        "five": 5
    }

    words = text.lower().split()

    for w in words:
        # Try numeric
        try:
            return float(w)
        except:
            pass

        # Try word numbers
        if w in word_to_num:
            return float(word_to_num[w])

    return None
# MAIN LOOP
# ----------------------------
def main():
    print("Speech Input Prototype Ready (Ctrl+C to exit)\n")

    while True:
        try:
            audio_file = record_audio()

            text = transcribe_audio(audio_file)
            print(f"\nYou said: {text}")

            command = parse_command(text)
            print(f"Parsed Command: {command}")

            time.sleep(1)

        except KeyboardInterrupt:
            print("\n Exiting...")
            break


if __name__ == "__main__":
    main()
