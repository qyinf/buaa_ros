#!/usr/bin/env python
# coding=utf-8

import rospy
import time
import multiprocessing
from team_3.srv import Base, BaseResponse, Conn, ConnResponse
from waterplus_map_tools.srv import GetNumOfWaypoints, GetWaypointByIndex, GetWaypointByName
import os
import io
import re
import signal
from xml.dom.minidom import parse
from util import terminate_process


class Waypoints:
    def __init__(self):
        rospy.wait_for_service("waterplus/get_num_waypoint")
        cliGetNum = rospy.ServiceProxy(
            "waterplus/get_num_waypoint", GetNumOfWaypoints)
        cliGetWPName = rospy.ServiceProxy(
            "waterplus/get_waypoint_name", GetWaypointByName)
        srvNum = cliGetNum.call()
        rospy.wait_for_service("waterplus/get_waypoint_index")
        cliGetWPIndex = rospy.ServiceProxy(
            "waterplus/get_waypoint_index", GetWaypointByIndex)
        srvl = 0
        self.sum = srvNum.num
        self.points = []
        self.dict = {}
        for i in range(srvNum.num):
            srvl = cliGetWPIndex(i)
            name = srvl.name
            self.points.append(srvl)
            self.dict[name] = srvl

    def getWaypointByName(self, target):
        rospy.wait_for_service("waterplus/get_num_waypoint")
        cliGetNum = rospy.ServiceProxy(
            "waterplus/get_num_waypoint", GetNumOfWaypoints)
        cliGetWPName = rospy.ServiceProxy(
            "waterplus/get_waypoint_name", GetWaypointByName)
        srvNum = cliGetNum.call()
        rospy.wait_for_service("waterplus/get_waypoint_index")
        cliGetWPIndex = rospy.ServiceProxy(
            "waterplus/get_waypoint_index", GetWaypointByIndex)
        srvl = 0
        self.sum = srvNum.num
        self.points = []
        self.dict = {}
        for i in range(srvNum.num):
            srvl = cliGetWPIndex(i)
            name = srvl.name
            self.points.append(srvl)
            self.dict[name] = srvl
        return self.dict[target]        
            

    def getNames(self):
        rospy.wait_for_service("waterplus/get_num_waypoint")
        cliGetNum = rospy.ServiceProxy(
            "waterplus/get_num_waypoint", GetNumOfWaypoints)
        cliGetWPName = rospy.ServiceProxy(
            "waterplus/get_waypoint_name", GetWaypointByName)
        srvNum = cliGetNum.call()
        rospy.wait_for_service("waterplus/get_waypoint_index")
        cliGetWPIndex = rospy.ServiceProxy(
            "waterplus/get_waypoint_index", GetWaypointByIndex)
        srvl = 0
        self.sum = srvNum.num
        self.points = []
        self.dict = {}
        for i in range(srvNum.num):
            srvl = cliGetWPIndex(i)
            name = srvl.name
            self.points.append(srvl)
            self.dict[name] = srvl
        return self.dict.keys()


class Mark:
    def __init__(self):
        rospy.init_node("mark")
        rospy.loginfo("mark start!")

        rospy.Service('/control/mark/edit', Base, self.edit)
        rospy.Service('/control/mark/save', Conn, self.save)

        self.pid = -1
        self.map_server_pid = -1

    # 编辑航点
    def edit(self, req):
        # 将当前地图的航点文件移到主目录
        mark_path = rospy.get_param("pkg_path") + '/marks/waypoints' + '.xml'
        map_path = rospy.get_param("pkg_path") + '/maps/map' + '.yaml'
        if not os.path.exists(map_path):
            return BaseResponse("地图不存在")
        if os.path.exists(mark_path):
            os.system("cp " + mark_path + " ~/waypoints.xml")

        if self.pid != -1:
            return BaseResponse("正在标注")

        def map_server_process():
            os.system("rosrun map_server map_server " + map_path)

        p = multiprocessing.Process(target=map_server_process)
        p.start()
        self.map_server_pid = p.pid

        def t():
            if rospy.get_param("simulate"):
                os.system("roslaunch team_3 sim_mark.launch")
            else:
                os.system("roslaunch team_3 robot_mark.launch")
                
        p = multiprocessing.Process(target=t)
        p.start()
        self.pid = p.pid
        s = Waypoints().getNames()
        return BaseResponse("开始编辑，已有航点"+ str(len(s)) + "个： " + ' '.join(s))

    def save(self, req):
        if req.arg in Waypoints().getNames():
            terminate_process(self.pid)
            terminate_process(self.map_server_pid)
            self.pid = -1
            self.map_server_pid = -1
            return ConnResponse("航点名重复，请重新进行航点标注")

        mark_path = rospy.get_param("pkg_path") + '/marks/waypoints' + '.xml'
        os.system('rosrun waterplus_map_tools wp_saver')
        os.system('mv ~/waypoints.xml ' + mark_path)

        # 修改xml
        # doc = parse(mark_path)
        # root = doc.documentElement
        # points = root.getElementsByTagName('Waypoint')
        # for p in points:
        #     if p.getElementsByTagName('Name')[0].childNodes[0].data == '1':
        #         p.getElementsByTagName('Name')[0].childNodes[0].data = req.arg
        # with open(mark_path, 'w') as f:
        #     doc.writexml(f, encoding='utf-8')
        with open(mark_path, "r") as f1, open("%s.bak" % mark_path, "w") as f2:
            for line in f1:
                f2.write(re.sub("<Name>1</Name>", "<Name>" + req.arg + "</Name>", line))
        os.remove(mark_path)
        os.rename("%s.bak" % mark_path, mark_path)

        terminate_process(self.pid)
        terminate_process(self.map_server_pid)
        self.pid = -1
        self.map_server_pid = -1

        return ConnResponse("航点" + req.arg + "保存成功")


if __name__ == '__main__':
    Mark()
    rospy.spin()
