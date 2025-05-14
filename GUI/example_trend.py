import tkinter as tk
from tkinter import ttk
import psutil
import time
from datetime import datetime
import math

gInputVal = 0
gOutputVal = 0
gTrendMode = 0

class AnalogTrend:
    def __init__(self, root):
        self.root = root
        self.root.title("System Monitor")
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

def main():
    root = tk.Tk()
    app = AnalogTrend(root)
    root.mainloop()

if __name__ == "__main__":
    main()