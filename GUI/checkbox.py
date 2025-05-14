# -*- coding : utf-8 -*-
u"""
GUI example using checkboxes
"""
import tkinter
from tkinter import messagebox

def button_push(event):
    edit_box.delete(0, tkinter.END)
def func_check(event):
    global val1
    global val2
    global val3
    text = ""
    if val1.get() == True:
        text += "first \n"
    else:
        text += "no 1 \n"
    if val2.get() == True:
        text += "second \n"
    else:
        text += "no 2 \n"
    if val3.get() == True:
        text += "third \n"
    else:
        text += "no 3 ん\n"
    edit_box.insert(tkinter.END, text)
    messagebox.showinfo("info", text)

if __name__ == "__main__":
    root = tkinter.Tk()
    root.title(u"GUI with check boxes")
    root.geometry("400x300")
    edit_box = tkinter.Entry(width=50)
    edit_box.insert(tkinter.END, "initial string text")
    edit_box.pack()
    button = tkinter.Button(text=u"clear text", width=30)
    button.bind("<Button-1>", button_push)
    button.pack()
    val1 = tkinter.BooleanVar()
    val2 = tkinter.BooleanVar()
    val3 = tkinter.BooleanVar()
    val1.set(False)
    val2.set(True)
    val3.set(False)
    checkbox1 = tkinter.Checkbutton(text=u"no1", variable=val1)
    checkbox1.pack()
    checkbox2 = tkinter.Checkbutton(text=u"no2", variable=val2)
    checkbox2.pack()
    checkbox3 = tkinter.Checkbutton(text=u"no3", variable=val3)
    checkbox3.pack()
    button2 = tkinter.Button(root, text=u"read checkboxes", width=50)
    button2.bind("<Button-1>", func_check)
    button2.pack()
    tkinter.mainloop()
	