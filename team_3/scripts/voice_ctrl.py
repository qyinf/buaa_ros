#!/usr/bin/env python3
# coding=utf-8
import sys
import os


def get_text_type(text):
    print("open")
    if "你是谁" in text:
        os.system('mpg123 /home/robot/catkin_ws/src/team_3/scripts/名字.mp3')
    if "值班医生" in text:
        os.system('mpg123 /home/robot/catkin_ws/src/team_3/scripts/值班医生.mp3')
    if "值班医生的电话" in text:
        os.system('mpg123 /home/robot/catkin_ws/src/team_3/scripts/值班医生电话.mp3')
    if "值班护士" in text:
        os.system('mpg123 /home/robot/catkin_ws/src/team_3/scripts/值班护士.mp3')
    if "值班护士的电话" in text:
        os.system('mpg123 /home/robot/catkin_ws/src/team_3/scripts/值班护士电话.mp3')
    if "卫生间" in text:
        os.system('mpg123 /home/robot/catkin_ws/src/team_3/scripts/卫生间.mp3')
    if "厕所" in text:
        os.system('mpg123 /home/robot/catkin_ws/src/team_3/scripts/卫生间.mp3')
    if "缴费大厅" in text:
        os.system('mpg123 /home/robot/catkin_ws/src/team_3/scripts/缴费大厅.mp3')


if __name__ == "__main__":
    # rospy.init_node("voicectrl")
    args = sys.argv[:]
    text = args[1]
    get_text_type(text)

