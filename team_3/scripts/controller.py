#!/usr/bin/env python3
# coding=utf-8

import os
import signal
import time
import multiprocessing
import rospy
import yaml
import psutil
import tkinter
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import String
from team_3.srv import Base, Conn, ConnResponse
from util import terminate_process
from tkinter import ttk
from tkinter import scrolledtext
from tkinter import Menu
from tkinter import Spinbox
from tkinter import messagebox as mBox
import serial
import serial.tools.list_ports
import time
import random

params = {}
controller = None
tkinterUI = None
logger = None

def is_serial_port_exist(port_name):
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        if port_name in port:
            return True
    return False


class Controller:

    def __init__(self):

        def launch_init():
            if params['simulate']:
                os.system("roslaunch team_3 sim_init.launch")
            else:
                os.system("roslaunch team_3 robot_init.launch")

        p = multiprocessing.Process(target=launch_init)
        p.start()
        self.init_pid = p.pid

        rospy.init_node("controller")

        for key in params:
            rospy.set_param(key, params[key])

        self.voice_pub = rospy.Publisher('/voice_input_control', String, queue_size=10)

    def create_map_start(self):
        client = rospy.ServiceProxy('/control/create_map/start', Base)
        rospy.wait_for_service('/control/create_map/start')
        resp = client('start')
        loginfo(resp.response)
        tkinterUI.insert_text(0, resp.response)

    def create_map_save(self, map_id=None):
        client = rospy.ServiceProxy('/control/create_map/save', Base)
        rospy.wait_for_service('/control/create_map/save')
        # if params['use_tkinter'] and map_id == None:
        # map_id = tkinterUI.t1.get()
        save_path = params['pkg_path'] + '/maps/map'
        resp = client(save_path)
        loginfo(resp.response)

    def thermometry(self):
        loginfo("开始测温，请将手放在测温传感器旁。")
        port_name = '/dev/ttyUSB2'
        if is_serial_port_exist(port_name):
            ser = serial.Serial("/dev/ttyUSB2", baudrate=9600, timeout=1)
            hex_data = b'\x01\x03\x00\x00\x00\x02\xC4\x0B'
            loginfo("正在测温，请勿移动。")
            ser.write(hex_data)
            r = ser.readline()
            if r[4] == 255:
                loginfo("温度传感器损坏，请联系医生。") 
            else:    
                t = (256*r[3] + r[4]) / 10
                l = "您的体温是：" + str(t) + "摄氏度。"
                loginfo(l)
            ser.close()
        else:
            loginfo("测温失败，请检查测温传感器是否连接好。")
        
    
    def edit_mark(self, map_id=None):
        client = rospy.ServiceProxy('/control/mark/edit', Base)
        rospy.wait_for_service('/control/mark/edit')
        # if params['use_tkinter'] and map_id == None:
        # map_id = tkinterUI.t1.get()
        # map_id = None
        resp = client(str(map_id))
        loginfo(resp.response)

    def save_mark(self, map_id=None, label=None):
        client = rospy.ServiceProxy('/control/mark/save', Conn)
        rospy.wait_for_service('/control/mark/save')
        if params['use_tkinter'] and map_id == None:
            # map_id = tkinterUI.t1.get()
            map_id = None
            label = tkinterUI.t1.get()
        resp = client("", map_id, label)
        loginfo(resp.response)

    def navigation_init(self, map_id=None):
        """选择一张地图进行导航初始化，调整机器人初始位置。

        Args:
            map_id (int): 地图ID
        """
        client = rospy.ServiceProxy('/control/navigation/init', Base)
        rospy.wait_for_service('/control/navigation/init')
        if params['use_tkinter'] and map_id == None:
            map_id = None
        resp = client(str(map_id))
        loginfo(resp.response)

    def navigation_begin(self, dst=None):
        client = rospy.ServiceProxy('/control/navigation/begin', Base)
        rospy.wait_for_service('/control/navigation/begin')
        if params['use_tkinter'] and dst == None:
            dst = tkinterUI.t2.get()
        resp = client(str(dst))
        loginfo(resp.response)

    def navigation_finish(self):
        client = rospy.ServiceProxy('/control/navigation/finish', Base)
        rospy.wait_for_service('/control/navigation/finish')
        resp = client('start')
        loginfo(resp.response)

    def grab(self):
        """在当前位置执行抓取
        """
        client = rospy.ServiceProxy('/control/arm', Base)
        rospy.wait_for_service('/control/arm')
        resp = client('grab')
        loginfo(resp.response)

    def pass_obj(self):
        """当前位置放下物品
        """
        client = rospy.ServiceProxy('/control/arm', Base)
        rospy.wait_for_service('/control/arm')
        resp = client('pass')
        loginfo(resp.response)

    def voice(self):
        loginfo("begin")
        vspace = String()
        vspace.data = " "
        self.voice_pub.publish(vspace)
        with open('help.txt', 'r') as file:
            content = file.read()
            loginfo(content)

    def exit(self):
        terminate_process(self.init_pid)
        if params['use_tkinter']:
            tkinterUI.window.destroy()
        exit(0)

class TkinterUI:
    def __init__(self, controller: Controller):
        self.window = tkinter.Tk()
        self.window.tk.call("source", "forest-light.tcl")
        ttk.Style().theme_use('forest-light')
        # default_font = font.nametofont("TkDefaultFont")
        # default_font.configure(size=100)
        # 一些样式
        # Create a Frame for input widgets

        # self.t1 = ttk.Entry(self.window)
        # self.t1.pack()
        # self.t2 = ttk.Entry(self.window)
        # self.t2.pack()

        # 新建四个页面
        tabControl = ttk.Notebook(self.window)
        tab1 = ttk.Frame(tabControl)  # Create a tab
        tabControl.add(tab1, text='建图功能')
        tab2 = ttk.Frame(tabControl)  # Create a tab
        tabControl.add(tab2, text='航点功能')
        tab3 = ttk.Frame(tabControl)  # Create a tab
        tabControl.add(tab3, text='服务模式')
        tab4 = ttk.Frame(tabControl)  # Create a tab
        tabControl.add(tab4, text='测温')
        tab5 = ttk.Frame(tabControl)  # Create a tab
        tabControl.add(tab5, text='音视频')
        tabControl.pack(expand=1, fill="both")

        # -------------------------------------tab1---------------------------------------#
        widgets_frame = ttk.LabelFrame(tab1)
        widgets_frame.grid(row=0, column=1, padx=10, pady=(30, 10), sticky="nsew", rowspan=3)
        widgets_frame.columnconfigure(index=0, weight=1)
        button = ttk.Button(widgets_frame, text="点击建图", command=controller.create_map_start)
        button.grid(row=6, column=0, padx=5, pady=10, sticky="nsew")

        # Accentbutton
        accentbutton = ttk.Button(widgets_frame, text="保存地图", style="Accent.TButton",
                                  command=controller.create_map_save)
        accentbutton.grid(row=7, column=0, padx=5, pady=10, sticky="nsew")
        
        label = ttk.Label(widgets_frame, text="提示：点击建图后使用手柄控制机器人移动建图")
        label.grid(row=8, column=0, pady=10, columnspan=2)

        tab1.rowconfigure(0, weight=1)
        tab1.columnconfigure(1, weight=1)

        # ---------------------------------tab2-------------------------------------------#
        widgets_frame2 = ttk.LabelFrame(tab2)
        widgets_frame2.grid(row=0, column=1, padx=10, pady=(30, 0), sticky="nsew", rowspan=3)
        widgets_frame2.columnconfigure(index=1, weight=1)

        t1_label = ttk.Label(widgets_frame2, text="航点名称")
        t1_label.grid(row=6, column=0, padx=30, pady=5, sticky="nsew")
        self.t1 = ttk.Entry(widgets_frame2)
        self.t1.grid(row=6, column=1, padx=10, pady=5, sticky="nsew")

        button = ttk.Button(widgets_frame2, text="编辑航点", command=controller.edit_mark)
        button.grid(row=7, column=0, padx=10, pady=10, sticky="nsew")
        accentbutton = ttk.Button(widgets_frame2, text="保存航点", style="Accent.TButton", command=controller.save_mark)
        accentbutton.grid(row=7, column=1, padx=10, pady=10, sticky="nsew")

        label = ttk.Label(widgets_frame2, text="提示：请一次只编辑和保存一个航点")
        label.grid(row=8, column=0, pady=10, columnspan=2)

        tab2.rowconfigure(0, weight=1)
        tab2.columnconfigure(1, weight=1)

        # -----------------------------------------------tab3-----------------------------------------------#
        widgets_frame3 = ttk.LabelFrame(tab3)
        widgets_frame3.grid(row=0, column=1, padx=10, pady=(30, 10), sticky="nsew", rowspan=3)
        widgets_frame3.columnconfigure(0, weight=1)
        widgets_frame3.columnconfigure(1, weight=1)
        
        label = ttk.Label(widgets_frame3, text="提示：请先进入服务模式调整机器人位置，再使用抓取放下和导航功能，在导航的时候请输入航点名称")
        label.grid(row=9, column=0, pady=10, columnspan=2)
        
        button = ttk.Button(widgets_frame3, text="进入服务模式", command=controller.navigation_init)
        button.grid(row=6, column=0, padx=5, pady=10, sticky="nsew")

        togglebutton = ttk.Checkbutton(widgets_frame3, text="退出服务模式", style="ToggleButton",
                                       command=controller.navigation_finish)
        togglebutton.grid(row=6, column=1, padx=5, pady=10, sticky="nsew")

        self.t2 = ttk.Entry(widgets_frame3)
        self.t2.grid(row=7, column=0, padx=10, pady=5, sticky="nsew")

        # Accentbutton
        accentbutton = ttk.Button(widgets_frame3, text="导航到目标点", style="Accent.TButton",
                                  command=controller.navigation_begin)
        accentbutton.grid(row=7, column=1, padx=5, pady=10, sticky="nsew")

        button = ttk.Button(widgets_frame3, text="抓取", command=controller.grab)
        button.grid(row=8, column=0, padx=5, pady=10, sticky="nsew")

        # Accentbutton
        accentbutton = ttk.Button(widgets_frame3, text="放下", style="Accent.TButton", command=controller.pass_obj)
        accentbutton.grid(row=8, column=1, padx=5, pady=10, sticky="nsew")
        
        tab3.rowconfigure(0, weight=1)
        tab3.columnconfigure(1, weight=1)

        # '''
        # --------------------------------------------------------------tab4--------------------------------------#
        widgets_frame4 = ttk.LabelFrame(tab4)
        widgets_frame4.grid(row=0, column=1, padx=10, pady=(30, 10), sticky="nsew", rowspan=3)
        widgets_frame4.columnconfigure(index=0, weight=1)
        button = ttk.Button(widgets_frame4, text="测温", command=controller.thermometry)
        button.grid(row=6, column=0, padx=5, pady=10, sticky="nsew")
        label = ttk.Label(widgets_frame4, text="提示：请点击按钮开始测温")
        label.grid(row=7, column=0, pady=10, columnspan=2)
        # Accentbutton
        # accentbutton = ttk.Button(widgets_frame4, text="放下", style="Accent.TButton", command=controller.pass_obj)
        # accentbutton.grid(row=7, column=0, padx=5, pady=10, sticky="nsew")
        tab4.rowconfigure(0, weight=1)
        tab4.columnconfigure(1, weight=1)
        # '''

        # --------------------------------------------------------------tab5--------------------------------------#
        widgets_frame5 = ttk.LabelFrame(tab5)
        widgets_frame5.grid(row=0, column=1, padx=10, pady=(30, 10), sticky="nsew", rowspan=3)
        widgets_frame5.columnconfigure(index=0, weight=1)
        button = ttk.Button(widgets_frame5, text="开始说话", command=controller.voice)
        button.grid(row=6, column=0, padx=5, pady=10, sticky="nsew")

        # Accentbutton
        accentbutton = ttk.Button(widgets_frame5, text="结束说话", style="Accent.TButton")
        accentbutton.grid(row=7, column=0, padx=5, pady=10, sticky="nsew")
        label = ttk.Label(widgets_frame5, text="提示：请点击按钮开始和结束说话")
        label.grid(row=8, column=0, pady=10, columnspan=2)

        tab5.rowconfigure(0, weight=1)
        tab5.columnconfigure(1, weight=1)

        # -----------------
        b5 = ttk.Button(self.window, text="退出", command=controller.exit)
        b5.pack(pady=10)

        l = tkinter.Label(self.window, text='输出信息', font=('微软雅黑', 10, 'bold'), width=500, justify='left',
                          anchor='w')
        l.pack()
        s1 = tkinter.Scrollbar(self.window)  # 设置垂直滚动条
        s2 = tkinter.Scrollbar(self.window, orient='horizontal')  # 水平滚动条
        s1.pack(side='right', fill='y')  # 靠右，充满Y轴
        s2.pack(side='bottom', fill='x')  # 靠下，充满x轴
        self.output = tkinter.Text(self.window, font=('Consolas', 9), undo=True, autoseparators=False,
                                   wrap='none', xscrollcommand=s2.set,
                                   yscrollcommand=s1.set)  # , state=DISABLED, wrap='none'表示不自动换行
        self.output.pack(fill='both', expand='yes')
        s1.config(command=self.output.yview)  # Text随着滚动条移动被控制移动
        s2.config(command=self.output.xview)

        widgets_frame = ttk.Frame(self.window, padding=(0, 0, 0, 10))

        self.window.title('医护机器人，小护')
        self.window.geometry('1000x600')
        self.window.option_add("*tearOff", False)
    

    def loop(self):
        self.window.mainloop()

    def log(self, text):
        now_time = time.strftime("%H:%M:%S")
        self.output.insert('end', '[' + now_time + '] ' + text + '\n')


def loginfo(text):
    if params['use_tkinter']:
        tkinterUI.log(text)
    logger.publish(text)


def loginfo_map(text):
    if params['use_tkinter']:
        tkinterUI.log(text)
    logger.publish(text)


if __name__ == '__main__':
    logger = rospy.Publisher('/control/logger', String, queue_size=10)
    pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(pkg_path + "/config/control.yaml", 'r') as file:
        params = yaml.load(file.read(), Loader=yaml.FullLoader)
        params['pkg_path'] = pkg_path

    controller = Controller()
    if params['use_tkinter']:
        tkinterUI = TkinterUI(controller)
        tkinterUI.loop()
    rospy.spin()

