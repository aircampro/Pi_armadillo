#
# Example of tkinter form which calculates time differences between start and end with a rest 
#
import tkinter
from datetime import datetime

def days_func():
    '''parse the date function '''
    import datetime
    t_delta = datetime.timedelta(hours=9)                        # choose your timezone here
    JST = datetime.timezone(t_delta, 'JST')                      # japan
    t_delta = datetime.timedelta(hours=-4)
    EST = datetime.timezone(t_delta, 'US/Eastern')               # east us
    t_delta = datetime.timedelta(hours=-8)
    PST = datetime.timezone(t_delta, 'US/Western')               # west us
    t_delta = datetime.timedelta(hours=-12)
    HST = datetime.timezone(t_delta, 'HST')                      # hawaii
    t_delta = datetime.timedelta(hours=5)
    IST = datetime.timezone(t_delta, 'IST')                      # india
    t_delta = datetime.timedelta(hours=0) 
    BST = datetime.timezone(t_delta, 'BST')                      # UK/Eire/Portugal
    # choose the timezone above to suit you
    now = datetime.datetime.now(BST)
    d_week = {'Sun':'Su', 'Mon':'Mo', 'Tue':'Tu', 'Wed':'We', 'Thu':'Th', 'Fri':'Fr', 'Sat':'Sa'}
    key = now.strftime('%a') 
    w = d_week[key] 
    days_today = now.date().strftime('%Y/%m/%d') 
    lbl_days = tkinter.Label(text=str(days_today + str(f' ({w})')))
    lbl_days.place(x=10, y=20)

def btn_click():
    total_value.delete(0, tkinter.END) 
    start = str(start_value.get())
    last = str(last_value.get())
    rest = str(rest_value.get())
    FMT = '%H:%M:%S'
    if int(start.split(":")[0]) > int(last.split(":")[0]):
        t_to_midnight = datetime.strptime("23:59:59", FMT) - datetime.strptime(start, FMT)
        t_after_midnight = datetime.strptime(last, FMT)
        ttm = str(t_to_midnight + t_after_midnight).split(" ")[1]                                      # remove the year part which was added 
        hms = ttm.split(":")                                                                           # add back the missing second
        s = int(hms[2])+1
        if s >= 60:
            s = 60 - s
            m = int(hms[1])+1
            if m >= 60:
                m = 60 - m
                h = int(hms[0])+1
            else:
                h = int(hms[0])
        else:
            m = int(hms[1])
            h = int(hms[0])
        ttm = str(h)+":"+str(m)+":"+str(s)      
        tdelta_cgange = ttm
    else:
        tdelta = datetime.strptime(last, FMT) - datetime.strptime(start, FMT)
        tdelta_cgange = str(tdelta)
    if len(str(tdelta_cgange).split(",")) > 1:                                                             # we have 23+ hours
        if int(str(tdelta_cgange).split(":")[0].split(",")[1]) == 23:
            hms = str(tdelta_cgange).split(":") 
            h = int(hms[0].split(",")[1])-1 
            tdelta_cgange = str(h)+":"+hms[1]+":"+hms[2]
            tdelta_1 = datetime.strptime(tdelta_cgange, FMT) - datetime.strptime(rest, FMT)  
            hms = str(tdelta_1).split(":")  
            h = int(hms[0])+1   
            tdelta_1 = str(h)+":"+hms[1]+":"+hms[2]        
        else:
            tdelta_1 = datetime.strptime(tdelta_cgange, FMT) - datetime.strptime(rest, FMT)
    else:
        tdelta_1 = datetime.strptime(tdelta_cgange, FMT) - datetime.strptime(rest, FMT)    
    total_value.insert(0, str(tdelta_1))

def btn_clear():
    """
    clears all the fields
    """
    start_value.delete(0, tkinter.END) 
    last_value.delete(0, tkinter.END)
    rest_value.delete(0, tkinter.END)
    total_value.delete(0, tkinter.END)


if __name__ == "__main__":
    tki = tkinter.Tk()
    tki.geometry('450x350')
    tki.title('Time Calcuklator')
    tki.configure(bg='SpringGreen2')

    lbl_massage = tkinter.Label(text='= Calculate total time between start and end =')
    lbl_massage.place(x=50, y=60)

    days_func()

    start_lbl = tkinter.Label(text='start time H:M:S   ')
    start_lbl.place(x=50, y=100)
    last_lbl = tkinter.Label(text='end time H:M:S   ')
    last_lbl.place(x=50, y=130)
    rest_lbl = tkinter.Label(text='rest time H:M:S   ')
    rest_lbl.place(x=50, y=160)
    total_lbl = tkinter.Label(text='Total Time   ')
    total_lbl.place(x=50, y=190)

    start_value = tkinter.Entry(width=30)
    start_value.place(x=170, y=100)
    last_value = tkinter.Entry(width=30)
    last_value.place(x=170, y=130)
    rest_value = tkinter.Entry(width=30)
    rest_value.place(x=170, y=160)
    total_value = tkinter.Entry(width=30)
    total_value.place(x=170, y=190)

    btn = tkinter.Button(tki, text='calculate', font=("MSゴシック", "10", "bold"), fg="red", relief= "ridge",command=btn_click)
    btn.place(x=170, y=220)
    btn = tkinter.Button(tki, text='clear', font=("MSゴシック", "10", "bold"), fg="red", relief= "ridge", command=btn_clear)
    btn.place(x=170, y=260)

    tki.mainloop() 
