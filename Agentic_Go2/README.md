Agent_move.py -- one command at a time

Agent_multistep.py -- adds a planning step to convert request into steps, multiple commands at a time

Agent_multistep_speech.py -- multiple steps, robot says steps as its being done

Set up:

start docker container from docker folder

create .env with OPENAI_API_KEY = "api-key-here"

create venv and pip install requirements

for speech only--
in anouther terminal, connected to same docker container
ros2 run speech_processor tts_node --ros-args \
-p api_key:="api-key-here" \
-p provider:="elevenlabs" \
-p playback:="robot"

run python3 Agent_move.py, Agent_multistep.py or Agent_multistep_speech.py

