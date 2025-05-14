#
# Example reads entry via button with progress bar and speed indicator using tkinter
#
import tkinter
from tkinter import ttk
import threading
import time

class Application(tkinter.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.pack()
        self.master.geometry("400x400")
        self.master.title(u"progress bar / bar graph indicator")
        self.create_widgets()
        self.funcs_run = 0
    def create_widgets(self):
        self.entry = tkinter.Entry(width=7)
        self.entry.insert(tkinter.END, '10')
        self.entry.place(x=140, y=30)
        self.set_button = tkinter.Button(text=u'Action', width=20)
        self.set_button.bind("<1>", self.call_func)
        self.set_button.place(x=100, y=50)
        self.si = ttk.Progressbar(root, orient="horizontal", length=200, mode="indeterminate")                   # speed indicator bar
        self.pb = ttk.Progressbar(root, orient="vertical", length=300, mode="determinate", maximum=100)          # bar gauge % full
        self.pb.place(x=70, y=100)
        self.si.place(x=90, y=100)
    def call_func(self, event):
        thread1 = threading.Thread(target=self.func, args=(int(self.entry.get()),))
        self.funcs_run += 1
        #print("-----> ",self.funcs_run)
        time.sleep(0.3)                               # sleep longer than loop time so the previous thread finishes (w/o mutex)
        thread1.start()
    def func(self, t:int):
        a = 100-t
        if a <= 0:
            a = 1
        self.si.start(a)
        #time.sleep(t)
        self.pb.configure(value=t)
        self.pb.update()
        st = time.time()
        while True:
            if (time.time() - st) > t:
                break
            if self.funcs_run > 1:
                break
            time.sleep(0.1)
        self.funcs_run -= 1
        #print("-----< ",self.funcs_run)
        self.si.stop()
        
if __name__ == "__main__":
    root = tkinter.Tk()
    app = Application(master=root)
    app.mainloop()