#
# example run a linux command from a tkinter pushbutton
#
import os, subprocess
import tkinter as tk
from tkinter import ttk, messagebox
import threading

#command = "ffmpeg movie1"                                                                    
command = "sleep 10"  

def start():
    progress = tk.Toplevel()
    progress.geometry("600x75")
    progress.title("processing...")
    info_text = tk.StringVar()
    info = tk.Label(progress, textvariable = info_text)
    info.pack(side = tk.TOP, anchor = tk.W)
    bar = ttk.Progressbar(progress,mode='indeterminate')
    bar.pack(side = tk.TOP, fill = tk.X)
    bar.start()
    def process():
        p = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,shell=True,universal_newlines=True)
        for line in p.stdout:
            info_text.set(line)
        try:
            outs, errs = p.communicate()
        except subprocess.TimeoutExpired:
            pass
        else:
            p.terminate()
            progress.destroy()
            warnwindow = messagebox.showwarning("Done", "task complete!")
    th1 = threading.Thread(target=process)
    th1.start()

if __name__ == "__main__":
    root = tk.Tk()
    button = tk.Button(root, text = "start", command = start)
    button.pack()
    root.mainloop()