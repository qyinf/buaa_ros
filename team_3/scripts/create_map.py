#!/usr/bin/env python
# coding=utf-8

import rospy
import time
import multiprocessing
from team_3.srv import Base, BaseResponse, Conn
import os
import signal
from PIL import Image
from util import terminate_process


class CreateMap:
    def __init__(self):
        rospy.init_node("create_map")
        rospy.loginfo("map creator start!")

        rospy.Service('/control/create_map/start', Base, self.start)
        rospy.Service('/control/create_map/save', Base, self.save)

        self.pid = -1

    # 开启建图
    def start(self, req):
        if self.pid != -1:  # 建图已经开始了
            return BaseResponse("建图正在运行")

        def t():
            if rospy.get_param("simulate"):
                os.system("roslaunch team_3 sim_create_map.launch")
            else:
                os.system("roslaunch team_3 robot_create_map.launch")
            
        p = multiprocessing.Process(target=t)
        p.start()
        self.pid = p.pid
        return BaseResponse("建图启动成功")
        
    # 保存地图：msg表示地图路径
    def save(self, req):
        map_name = req.request      # 没有后缀
        os.system('rosrun map_server map_saver -f %s' % map_name)

        # 先转为png，然后crop
        os.system('convert ' + map_name + '.pgm ' + map_name + '.png')
        Image.open(map_name + '.png').crop((1850, 1850, 2150, 2150)).save(map_name + '.png')

        terminate_process(self.pid)
        self.pid = -1
        return BaseResponse("地图保存成功")


if __name__ == '__main__':
    CreateMap()
    rospy.spin()
