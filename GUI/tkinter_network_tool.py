# network sniffing tool using tkinter
#
import nmap
from netaddr import *
from icmplib import *
import tkinter as tk
import tkinter.ttk as ttk

# command to execute
tab_list = [' nmap: -sS, -sA, -sN, -sU ', ' ping for subnet ']

# execute nmap
def ExecNmap(ip, num, arg, out):
    nm = nmap.PortScanner()
    scan_result = nm.scan(hosts = ip, ports = num, arguments = arg)
    out = out + nm.command_line() + '\n'
    #print(scan_result['scan'])
    target = next(iter(scan_result['scan'])) # for domain name ('ip' cannot be used.)
    if arg == '-sU':
        detail = scan_result['scan'][target]['udp']
    else:
        detail = scan_result['scan'][target]['tcp']
    first = next(iter(detail)) # 'num' does not work
    out = out + ' state: ' + detail[first]['state'] \
        + '\t\t\treason: ' + detail[first]['reason'] + '\n\n'
    return out

# execute multi-ping
def ExecPing(arg):
    ipNet = IPNetwork(arg)
    ipList=list(ipNet)
    Len = len(ipList)
    target = []
    for ip in ipList[1:Len-1]:
        target.append(str(ip))
    hosts = multiping(target, count=3)
    return hosts

# Function for click OK
def ok_clk(frm, arg):
    ip = frm.entry1.get()                          # e.g. 10.0.3.100/28 for arg=1 form and www.google.com 28 for form arg=0
    if arg == 0:
        num = frm.entry2.get()
        out = ''
        out = ExecNmap(ip, num, '-sS', out)
        out = ExecNmap(ip, num, '-sA', out)
        out = ExecNmap(ip, num, '-sN', out)
        out = ExecNmap(ip, num, '-sU', out)
    else:
        hosts = ExecPing(ip)
        out = 'ping to ' + ip + '\n'
        for host in hosts:
            if host.is_alive:
                out = out + ' ' + host.address + '\tis up' + '\n'
    frm.text.config(state='normal')   # editable
    frm.text.insert(tk.END, out)
    frm.text.config(state='disabled') # not editable
    frm.text.see('end')
    return

# Create content of tab
def create_content(frm, arg):
    frm1 = tk.Frame(frm)
    frm2 = tk.Frame(frm)
    frm1.pack()
    frm2.pack()
    frm.entry1 = ttk.Entry(frm1, width=20)
    frm.entry1.grid(row=0, column=1)
    if arg == 0:
        label = ttk.Label(frm1, text='Enter IP address and Port Number')
        frm.entry2 = ttk.Entry(frm1, width=10)
        frm.entry2.grid(row=0, column=2)
    else:
        label = ttk.Label(frm1, text='Enter Network (ex 192.168.1.0/24)')
    label.grid(row=0, column=0)
    okBtn = ttk.Button(frm1, text='OK', command=lambda:ok_clk(frm, arg))
    okBtn.grid(row=0, column=3)
    frm.text = tk.Text(frm2, width=80, height=40)
    ysc = tk.Scrollbar(frm2, orient=tk.VERTICAL, command=frm.text.yview)
    frm.text["yscrollcommand"] = ysc.set
    ysc.pack(side=tk.RIGHT, fill="y")
    frm.text.config(state='disabled') # not editable
    frm.text.pack()
    return

# Start of main program
# root main window
root = tk.Tk()
root.title("Port Check & Multi Ping")
root.geometry("600x400")

# Create an instance of ttk style
fgcolor = "lightskyblue2"
bgcolor = "gray80"
style = ttk.Style()
style.theme_create("style1", parent="alt", settings={
        "TNotebook.Tab": {
            "configure": {"background": bgcolor },
            "map":       {"background": [("selected", fgcolor)],
                          } } } )
style.theme_use("style1")

# Create Notebook Widget
note = ttk.Notebook(root)

# Create tab
tab0 = tk.Frame(note)
tab1 = tk.Frame(note)

# Add tab
note.add(tab0, text=tab_list[0])
note.add(tab1, text=tab_list[1])

# Create content of tab
create_content(tab0, 0)
create_content(tab1, 1)

# Locate tab
note.pack(expand=True, fill='both')

# main loop
root.mainloop()