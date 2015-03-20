# Google Text-to-Speech node
Uses an undocumented public API for Google's text-to-speech engine, which sounds significantly better than the default Festival voices.
This relies on having access to the internet.
In the future, we'll want to use a system like Festival which doesn't require internet access, maybe looking into better-sounding voices.

## Installation
```bash
sudo apt-get install sox
sudo apt-get install libsox-fmt-mp3
```

## Usage
```bash
rosrun google_tts speak.py
rosrun sound_play soundplay_node.py
rostopic pub -1 /google_tts std_msgs/String '{data: "Hello world"}'
```