#!/usr/bin/env python
# coding=utf-8

import tkinter
from tkinter import ttk
from tkinter import scrolledtext
from tkinter import Menu
from tkinter import Spinbox
from tkinter import messagebox as mBox


class TkinterUI:
    def __init__(self):
        self.window = tkinter.Tk()
        self.window.tk.call("source", "forest-light.tcl")
        ttk.Style().theme_use('forest-light')
        # 一些样式
        # Create a Frame for input widgets
        

        # 新建四个页面
        tabControl = ttk.Notebook(self.window)
        tab1 = ttk.Frame(tabControl)  # Create a tab
        tabControl.add(tab1, text='建图功能')
        tab2 = ttk.Frame(tabControl)  # Create a tab
        tabControl.add(tab2, text='航点功能')
        tab3 = ttk.Frame(tabControl)  # Create a tab
        tabControl.add(tab3, text='服务模式')
        tab4 = ttk.Frame(tabControl)  # Create a tab
        tabControl.add(tab4, text='物品抓取')
        tab5 = ttk.Frame(tabControl)  # Create a tab
        tabControl.add(tab5, text='音视频')
        tabControl.pack(expand=1, fill="both")

        # -------------------------------------tab1---------------------------------------#
        widgets_frame = ttk.LabelFrame(tab1)
        widgets_frame.grid(row=0, column=1, padx=10, pady=(30, 10), sticky="nsew", rowspan=3)
        widgets_frame.columnconfigure(index=0, weight=1)
        button = ttk.Button(widgets_frame, text="点击建图")
        button.grid(row=6, column=0, padx=10, pady=10, sticky="nsew")

        # Accentbutton
        accentbutton = ttk.Button(widgets_frame, text="保存地图", style="Accent.TButton")
        accentbutton.grid(row=7, column=0, padx=10, pady=10, sticky="nsew")

        tab1.rowconfigure(0, weight=1)
        tab1.columnconfigure(1, weight=1)


        # ---------------------------------tab2-------------------------------------------#
        widgets_frame2 = ttk.LabelFrame(tab2)
        widgets_frame2.grid(row=0, column=1, padx=10, pady=(30, 0), sticky="nsew", rowspan=3)
        widgets_frame2.columnconfigure(index=1, weight=1)
        button = ttk.Button(widgets_frame2, text="编辑航点")
        button.grid(row=7, column=0, padx=10, pady=10, sticky="nsew")

        # Accentbutton
        accentbutton = ttk.Button(widgets_frame2, text="保存航点", style="Accent.TButton")
        accentbutton.grid(row=7, column=1, padx=10, pady=10, sticky="nsew")

        t1_label = ttk.Label(widgets_frame2, text="航点名称")
        t1_label.grid(row=6, column=0, padx=30, pady=5, sticky="nsew")
        t1 = ttk.Entry(widgets_frame2)
        t1.grid(row=6, column=1, padx=10, pady=5, sticky="nsew")

        tab2.rowconfigure(0, weight=1)
        tab2.columnconfigure(1, weight=1)


        # -----------------------------------------------tab3-----------------------------------------------#
        widgets_frame3 = ttk.LabelFrame(tab3)
        widgets_frame3.grid(row=0, column=1, padx=10, pady=(30, 30), sticky="nsew", rowspan=3)
        widgets_frame3.columnconfigure(index=1, weight=1)
        button = ttk.Button(widgets_frame3, text="进入服务模式")
        button.grid(row=6, column=0, padx=5, pady=10, sticky="nsew")
        togglebutton = ttk.Checkbutton(widgets_frame3, text="退出服务模式", style="ToggleButton")
        togglebutton.grid(row=6, column=1, padx=5, pady=10, sticky="nsew")

        
        t2 = ttk.Entry(widgets_frame3)
        t2.grid(row=7, column=0, padx=10, pady=5, sticky="nsew")
        # Accentbutton
        accentbutton = ttk.Button(widgets_frame3, text="导航到目标点", style="Accent.TButton")
        accentbutton.grid(row=7, column=1, padx=5, pady=10, sticky="nsew")

        t2_label = ttk.Label(widgets_frame3, text="取药点")
        t2_label.grid(row=9, column=0, padx=30, pady=5, sticky="nsew")
        t3 = ttk.Entry(widgets_frame3)
        t3.grid(row=9, column=1, padx=10, pady=5, sticky="nsew")

        t3_label = ttk.Label(widgets_frame3, text="放置点")
        t3_label.grid(row=11, column=0, padx=30, pady=5, sticky="nsew")
        t4 = ttk.Entry(widgets_frame3)
        t4.grid(row=11, column=1, padx=10, pady=5, sticky="nsew")

        accentbutton = ttk.Button(widgets_frame3, text="抓取导航一体化", style="Accent.TButton")
        accentbutton.grid(row=13, column=1, padx=5, pady=10, sticky="nsew")

        tab3.rowconfigure(0, weight=1)
        tab3.columnconfigure(1, weight=1)

        # --------------------------------------------------------------tab4--------------------------------------#
        widgets_frame4 = ttk.LabelFrame(tab4)
        widgets_frame4.grid(row=0, column=1, padx=10, pady=(30, 10), sticky="nsew", rowspan=3)
        widgets_frame4.columnconfigure(index=0, weight=1)
        button = ttk.Button(widgets_frame4, text="抓取")
        button.grid(row=6, column=0, padx=5, pady=10, sticky="nsew")

        # Accentbutton
        accentbutton = ttk.Button(widgets_frame4, text="放下", style="Accent.TButton")
        accentbutton.grid(row=7, column=0, padx=5, pady=10, sticky="nsew")

        tab4.rowconfigure(0, weight=1)
        tab4.columnconfigure(1, weight=1)

        # -----------------
        b5 = ttk.Button(self.window, text="退出")
        b5.pack(pady=10)
        # self.t1 = ttk.Entry(self.window)
        # self.t1.pack()
        # self.t2 = ttk.Entry(self.window)
        # self.t2.pack()

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


if __name__ == '__main__':
    tkinterUI = TkinterUI()
    tkinterUI.loop()
