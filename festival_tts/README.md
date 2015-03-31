# Festival Text-to-Speech node
Speaks out any text sent to the corresponding topic.

## Usage
```bash
rosrun festival_tts speak.py
rosrun sound_play soundplay_node.py
rostopic pub -1 /festival_tts std_msgs/String '{data: "Hello world"}'
```