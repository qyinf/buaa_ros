#!/usr/bin/env python2
# coding=utf-8
from tkinter import ttk
import tkinter
import time

def init_voice():
    loginfo1("开始说话")

def get_text_type(text):
    init_voice()
    print(text)
    if "谁" in text:
        loginfo1("我是你爹")
    elif "药" in text:
        loginfo1("切克闹")

class Controller:
    def exit(self):
        tkinterUI.window.destroy()
        exit(0)

class TkinterUI:
    def __init__(self, controller: Controller):
        self.window = tkinter.Tk()
        self.window.tk.call("source", "forest-light.tcl")
        ttk.Style().theme_use('forest-light')

        # 新建四个页面
        tabControl = ttk.Notebook(self.window)
        tab5 = ttk.Frame(tabControl)  # Create a tab
        tabControl.add(tab5, text='音视频')
        tabControl.pack(expand=1, fill="both")

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

        self.window.title('controller')
        self.window.geometry('400x600')
        self.window.option_add("*tearOff", False)

    def loop(self):
        self.window.mainloop()

    def log(self, text):
        now_time = time.strftime("%H:%M:%S")
        self.output.insert('end', '[' + now_time + '] ' + text + '\n')


def loginfo1(text):
    print(text)
    tkinterUI.log(text)

if __name__ == "__main__":
    text = "你是谁"
    controller = Controller()
    tkinterUI = TkinterUI(controller)
    get_text_type(text)
    tkinterUI.loop()
