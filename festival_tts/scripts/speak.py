#!/usr/bin/env python

import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

class FestivalTTS:
  def __init__(self):
    rospy.init_node('festival_tts')
    self._client = SoundClient()
    self.voice = 'voice_cmu_us_slt_arctic_hts'
    rospy.Subscriber('festival_tts', String, self._response_callback)
    rospy.spin()

  def _response_callback(self, data):
    self._client.say(data.data, self.voice)

def main():
  tts = FestivalTTS()

if __name__ == '__main__':
  main()
