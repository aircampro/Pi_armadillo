#
# Simple Editor in TK to replace tabs with space chars
#
import tkinter
import tkinter.filedialog

# functions 
def load_text():
    typ = [("Text","*.txt"),("Python","*.py")] 
    fn = tkinter.filedialog.askopenfilename(filetypes=typ)
    if fn != "":
        f = None
        try:
            f = open(fn, 'r', encoding= "utf-8")
            te.delete("1.0", "end")
            te.insert("1.0", f.read())
        except:
            f = open(fn, 'r', encoding= "shift-jis")
            te.delete("1.0", "end")
            te.insert("1.0", f.read())
        finally:
            if f != None:
                f.close()
def save_text():
    typ = [("Text", "*.txt")]
    fn = tkinter.filedialog.asksaveasfilename(filetypes=typ)
    if fn != "":
        if fn[-4:] != ".txt":
            fn = fn + ".txt"
        with open(fn, 'w', encoding="utf-8") as f:
            f.write(te.get("1.0","end-1c"))
def col_black():
    te.configure(bg="black", fg="white", insertbackground="white")
def col_white():
    te.configure(bg="white", fg="black", insertbackground="black")

# list of chars to replace
TABS = ["\t"]

# list of replacement chars
SPA = [" "]

# processes replacement from TABS list to SPA list
def auto_proc():
    txt = te.get("1.0", "end-1c")
    print("got ",txt)
    for i in range(len(TABS)):
        txt = txt.replace(TABS[i], SPA[i])
    te.delete("1.0", "end")
    print("got ",txt)
    te.insert("1.0", txt)

# main window
root = tkinter.Tk()
root.title("simple editor")
fr = tkinter.Frame()
fr.pack(expand=True, fill=tkinter.BOTH)
te = tkinter.Text(fr, width=80, height=30)
sc = tkinter.Scrollbar(fr, orient = tkinter.VERTICAL, command=te.yview)
sc.pack(side=tkinter.RIGHT, fill=tkinter.Y)
te.pack(expand=True, fill=tkinter.BOTH)
te[ "yscrollcommand" ] = sc.set
mbar = tkinter.Menu()
mcom = tkinter.Menu(mbar, tearoff = 0)
mcom.add_command(label="Load File", command=load_text)
mcom.add_separator()
mcom.add_command(label="Save File", command=save_text)
mbar.add_cascade(label="File Commands", menu=mcom)
mcom2 = tkinter.Menu(mbar, tearoff = 0)
mcom2.add_command(label="Black", command=col_black)
mcom2.add_command(label="White", command=col_white)
mbar.add_cascade(label="BG Color", menu=mcom2)
mcom3 = tkinter.Menu(mbar, tearoff = 0)
mcom3.add_command(label="Tabs to Space", command=auto_proc)
mbar.add_cascade(label="Conversion Macro", menu=mcom3)
root["menu"] = mbar
root.mainloop()
