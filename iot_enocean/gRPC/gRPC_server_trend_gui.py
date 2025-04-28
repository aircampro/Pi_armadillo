#
# Trend using tkinter which is updated from gRPC client calls in real-time
#
# pip install grpcio grpcio-tools
#
import grpc
from concurrent import futures
import example_pb2
import example_pb2_grpc
import tkinter as tk
from tkinter import ttk
import psutil
import time
from datetime import datetime
import math
import threading
import ctypes

# program Globals to be changed over the gRPC and viewed on the GUI
#
gInputVal = 0
gOutputVal = 0
gTrendMode = 0

class AnalogTrend:
    def __init__(self, root):
        self.root = root
        self.root.title("Configurable gRPC Trend GUI")
        self.root.geometry("800x600")
        self.root.configure(bg='#2b2b2b')

        style = ttk.Style()
        style.theme_use('default')
        style.configure('TButton', 
                       padding=6, 
                       relief="flat",
                       background="#4CAF50",
                       foreground="white")
        style.configure('TLabel', 
                       background='#2b2b2b',
                       foreground='white')
        
        control_frame = tk.Frame(root, bg='#2b2b2b')
        control_frame.pack(fill='x', padx=10, pady=5)
        
        ttk.Label(control_frame, 
                 text="Update Interval:",
                 style='TLabel').pack(side='left', padx=5)
        
        self.interval_var = tk.StringVar(value='1')
        interval_menu = ttk.OptionMenu(control_frame, 
                                     self.interval_var,
                                     '1',
                                     '1', '2', '5', '10',
                                     command=self.change_interval)
        interval_menu.pack(side='left', padx=5)
        
        ttk.Label(control_frame, 
                 text="seconds",
                 style='TLabel').pack(side='left')
        
        self.running = True
        self.toggle_button = ttk.Button(control_frame,
                                      text="Stop",
                                      command=self.toggle_update)
        self.toggle_button.pack(side='left', padx=10)
        
        self.canvas = tk.Canvas(root,
                              bg='#1E1E1E',
                              width=750,
                              height=500,
                              highlightthickness=0)
        self.canvas.pack(padx=10, pady=5, fill='both', expand=True)
        
        self.status_var = tk.StringVar()
        status_label = ttk.Label(root,
                               textvariable=self.status_var,
                               style='TLabel')
        status_label.pack(pady=5)
        
        self.cpu_history = []
        self.memory_history = []
        self.max_points = 60  
        
        self.update_data()
        
    def update_data(self):
        """updates the trend with data """
        if not self.running:
            return
        
        try:
            # show either cpu% and memory or what the gRPC server has as global variables
            if gTrendMode == 1:
                cpu_percent = psutil.cpu_percent()
                memory_percent = psutil.virtual_memory().percent
            else:
                cpu_percent = gInputVal
                memory_percent = gOutputVal

            # append the new values to the trend				
            self.cpu_history.append(cpu_percent)
            self.memory_history.append(memory_percent)
            
            if len(self.cpu_history) > self.max_points:
                self.cpu_history.pop(0)
                self.memory_history.pop(0)
            
            self.draw_graph()
            
            self.status_var.set(f"Last update: {datetime.now().strftime('%H:%M:%S')}")
            
            interval = int(self.interval_var.get()) * 1000
            self.root.after(interval, self.update_data)
            
        except Exception as e:
            self.status_var.set(f"Error: {str(e)}")
            self.root.after(1000, self.update_data)
    
    def draw_graph(self):
        """draw trend graph"""
        self.canvas.delete('all')  
        
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        padding = 40
        
        self.draw_grid(width, height, padding)
        
        # if there is data draw it
        if self.cpu_history:
            # draw the 2 trend lines
            self.draw_line(self.cpu_history, '#4CAF50', width, height, padding)
            self.draw_line(self.memory_history, '#2196F3', width, height, padding)
        
        current_cpu = self.cpu_history[-1] if self.cpu_history else 0
        current_memory = self.memory_history[-1] if self.memory_history else 0
        
        self.canvas.create_text(10, 20,
                              text=f"CPU: {current_cpu:.1f}%",
                              fill='#4CAF50',
                              anchor='w')
        self.canvas.create_text(120, 20,
                              text=f"Memory: {current_memory:.1f}%",
                              fill='#2196F3',
                              anchor='w')
    
    def draw_grid(self, width, height, padding):
        """draws grid """

        for i in range(11): 
            y = padding + (height - 2 * padding) * (10 - i) / 10

            self.canvas.create_line(padding, y,
                                  width-padding, y,
                                  fill='#333333',
                                  dash=(2, 4))

            self.canvas.create_text(padding - 5, y,
                                  text=f"{i * 10}%",
                                  fill='white',
                                  anchor='e')
        for i in range(7):  
            x = padding + (width - 2 * padding) * i / 6

            self.canvas.create_line(x, padding,
                                  x, height-padding,
                                  fill='#333333',
                                  dash=(2, 4))

            self.canvas.create_text(x, height-padding+5,
                                  text=f"-{60-i*10}s",
                                  fill='white',
                                  anchor='n')
    
    def draw_line(self, data, color, width, height, padding):
        """draw line"""
        if not data:
            return
            
        points = []
        chart_width = width - 2 * padding
        chart_height = height - 2 * padding
        
        x_step = chart_width / (len(data) - 1) if len(data) > 1 else 0
        
        for i, value in enumerate(data):
            x = padding + (i * x_step)
            y = height - padding - (value / 100 * chart_height)
            points.extend([x, y])
        
        if len(points) >= 4:
            self.canvas.create_line(points,
                                  fill=color,
                                  width=2,
                                  smooth=True)
    
    def change_interval(self, _):
        """change interval"""
        if self.running:
            self.toggle_update()  
            self.toggle_update()  
    
    def toggle_update(self):
        """toggle start/stop button"""
        self.running = not self.running
        if self.running:
            self.toggle_button.configure(text="Stop")
            self.update_data()
        else:
            self.toggle_button.configure(text="Start")

def main_trend_gui():
    root = tk.Tk()
    app = AnalogTrend(root)
    root.mainloop()

# Class wrapper for threading 
# you can also kill task e.g. if op_task.is_alive(): op_task.raise_exception()
#
class twe(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        return

    def run(self):
        self._target(*self.args, **self.kwargs)

    def get_id(self):
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id

    def raise_exception(self):
        thread_id = self.get_id()
        resu = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), ctypes.py_object(SystemExit))
        if resu > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), 0)
            print('Failure in raising exception')
            
class Greeter(example_pb2_grpc.GreeterServicer):
    def gRPCCmdLine(self, request, context):
        global gInputVal, gOutputVal
        try:        
            cmd = str(request.name).split(":")[0]
            val = str(request.name).split(":")[1]
            if cmd == "ADD":
                gInputVal += float(val)
                gTrendMode = 0
            elif cmd == "SUB":
                gInputVal -= float(val)
                gTrendMode = 0
            elif cmd == "REPLACE":
                gInputVal = float(val)
                gTrendMode = 0
            elif cmd == "BOTH":
                gInputVal = float(val)
                gOutputVal = float(str(request.name).split(":")[2])
                gTrendMode = 0
            elif cmd == "SYS":
                gTrendMode = 1		
        except Exception as e:
            print("error occurred : ",e)
        return example_pb2.gRPCReply(message=f"You sent to gRPC server, {request.name}!")

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    example_pb2_grpc.add_GreeterServicer_to_server(Greeter(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    server.wait_for_termination()

if __name__ == '__main__':
    gui_trend1_task = twe(name = 'Thread Trend', target=main_trend_gui, args=(), kwargs={})
    gui_trend1_task.start()
    serve()
    if gui_trend1_task.is_alive(): gui_trend1_task.raise_exception()
    gui_trend1_task.join()