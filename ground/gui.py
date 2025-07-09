import tkinter as tk
from tkinter import ttk
from tcp_client import TCPClient
import threading 
import time

firstTimeConnecting = True

# create tcp server
tcp = TCPClient()
tcp.connect() # connect to portenta
    
def button(var,key,entry):
    msg = "TUNE,"
    err = False
    j = var.get().strip()
    if(j != ""):
        try:
            val = float(j)
            if(val > 10):
                # print("Too large, the value shouldn't be larger than 10, please retry")
                err = True
            else:
                msg += f"{key}={val:.3f},"
        except ValueError:
            # print("Not a number or float.")
            err = True
    entry.delete(0,tk.END)
    if(not err and len(msg) > 5):
        threading.Thread(target=send_and_update, args=(msg,), daemon=True).start()
        

def send_all():
    msg = "TUNE,"
    for key,var in user_set.items():
        j = var.get().strip()
        if(j == ""):
            # print("Since this entry is empty, it's data will not be changed.")
            continue # skip empty entries
        try:
            val = float(j)
            if(val > 10):
                # print(f"This value is too large for {key}:{val}.")
                continue # skip past
            else:   
                msg += f"{key}={val:.3f},"
        except ValueError:
            print("Only numbers or decimals allowed.")
    if len(msg) > 5:
        threading.Thread(target=send_and_update, args=(msg,), daemon=True).start()

    # Clear all entry fields
    for var in user_set.values():
        var.set("")

def update_curr_gains():
    # first get the current gains by sending a request
    gains_dict = tcp.request_data()
    if not gains_dict:
        # print("No data received to update.")
        return
    for key,val in gains_dict.items():
        if key in curr_set:
            # print(f"Current value : {curr_set[key].get()}",end=", ")
            # print(val)
            if float(val):
                curr_set[key].set(val)
            # print(f"Updated to: {curr_set[key].get()}")


def send_and_update(msg):
    tcp.send_data(msg)
    update_curr_gains()     

def quit():
    app.quit()
    app.destroy()

def wait_for_connect_and_upd():
    print("Waiting for connection...")
    tcp.connected_event.wait() # block until connected event is set
    print("Client connected. Waiting 1 second before requesting gains...")
    time.sleep(1) # give Portenta time to connect with Nano
    update_curr_gains()


# create app
app = tk.Tk()

app.title("Tuner")

# variables
btn_text = tk.StringVar(value="Send")
# buttons = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
# btns = {key : tk.StringVar(value="Send") for key in buttons}

keys = ['pp','pi','pd','rp','ri','rd','yp','yi','yd','app','api','apd','arp','ari','ard']
user_set = {key : tk.StringVar(value="") for key in keys}
curr_set = {key : tk.StringVar(value="0") for key in keys}
# create style object
s = ttk.Style()

# default theme for all frames
s.configure('TFrame',background='green')
s.configure('TButton',font="Calibri 16 bold")

ttk.Label(master=app,text="Rate PIDs",font="Calibri 18 bold",relief="raised").grid(row=0,column=0,padx=10,pady=10)
ttk.Label(master=app,text="Angle PIDs",font="Calibri 18 bold",relief='raised').grid(row=0,column=1,padx=10,pady=10)


# pitch
s.configure('Frame1.TFrame',background="Cyan")
p_frame = ttk.Frame(master = app,borderwidth=2,relief='raised',style='Frame1.TFrame')
p_frame.grid(row=1,column=0,padx=10,pady=10)

ttk.Label(master=p_frame,text="Pitch",font="Calibri 16 bold",background='cyan').grid(row=0,column=1,padx=10,pady=10)
ttk.Label(master=p_frame,text="P",font="Calibri 16 bold",background='cyan').grid(row=1,column=0,padx=10,pady=10)
entry1 = ttk.Entry(master=p_frame,textvariable=user_set['pp'])
entry1.grid(row=1,column=1,padx=10,pady=10)
ttk.Label(master=p_frame,text="Current: ",font="Calibri 16 bold",background="Cyan").grid(row=1,column=2,padx=10,pady=10)
ttk.Label(master=p_frame,textvariable=curr_set['pp'],font="Calibri 16 bold",background="Cyan").grid(row=1,column=3,padx=10,pady=10)
ttk.Button(master=p_frame,text="Send",textvariable=btn_text,command=lambda v = user_set['pp']: button(v,keys[0],entry1),style='TButton').grid(row=1,column=4,padx=10,pady=10)

ttk.Label(master=p_frame,text="I",font="Calibri 16 bold",background="Cyan").grid(row=2,column=0,padx=10,pady=10)
entry2 = ttk.Entry(master=p_frame,textvariable=user_set['pi'])
entry2.grid(row=2,column=1,padx=10,pady=10)
ttk.Label(master=p_frame,text="Current: ",font="Calibri 16 bold",background="Cyan").grid(row=2,column=2,padx=10,pady=10)
ttk.Label(master=p_frame,textvariable=curr_set['pi'],font="Calibri 16 bold",background="Cyan").grid(row=2,column=3,padx=10,pady=10)
ttk.Button(master=p_frame,textvariable=btn_text,command=lambda v=user_set['pi']: button(v,keys[1],entry2),style='TButton').grid(row=2,column=4,padx=10,pady=10)

ttk.Label(master=p_frame,text="D",font="Calibri 16 bold",background="Cyan").grid(row=3,column=0,padx=10,pady=10)
entry3 = ttk.Entry(master=p_frame,textvariable=user_set['pd'])
entry3.grid(row=3,column=1,padx=10,pady=10)
ttk.Label(master=p_frame,text="Current: ",font="Calibri 16 bold",background="Cyan").grid(row=3,column=2,padx=10,pady=10)
ttk.Label(master=p_frame,textvariable=curr_set['pd'],font="Calibri 16 bold",background="Cyan").grid(row=3,column=3,padx=10,pady=10)
ttk.Button(master=p_frame,textvariable=btn_text,command=lambda v=user_set['pd']: button(v,keys[2],entry3),style='TButton').grid(row=3,column=4,padx=10,pady=10)



# roll
s.configure('Frame2.TFrame',background='lightgreen')
r_frame = ttk.Frame(master=app,borderwidth=2,relief='raised',style='Frame2.TFrame')
r_frame.grid(row=2,column=0,padx=10,pady=10)

ttk.Label(master=r_frame,text="Roll",font="Calibri 16 bold",background='lightgreen').grid(row=0,column=1,padx=10,pady=10)

ttk.Label(master=r_frame,text="P",font="Calibri 16 bold",background='lightgreen').grid(row=1,column=0,padx=10,pady=10)
entry4 = ttk.Entry(master=r_frame,textvariable=user_set['rp'])
entry4.grid(row=1,column=1,padx=10,pady=10)
ttk.Label(master=r_frame,text="Current: ",font="Calibri 16 bold",background='lightgreen').grid(row=1,column=2,padx=10,pady=10)
ttk.Label(master=r_frame,textvariable=curr_set['rp'],font="Calibri 16 bold",background="lightgreen").grid(row=1,column=3,padx=10,pady=10)
ttk.Button(master=r_frame,textvariable=btn_text,command=lambda v=user_set['rp']: button(v,keys[3],entry4),style='TButton').grid(row=1,column=4,padx=10,pady=10)


ttk.Label(master=r_frame,text="I",font="Calibri 16 bold",background='lightgreen').grid(row=2,column=0,padx=10,pady=10)
entry5 = ttk.Entry(master=r_frame,textvariable=user_set['ri'])
entry5.grid(row=2,column=1,padx=10,pady=10)
ttk.Label(master=r_frame,text="Current: ",font="Calibri 16 bold",background='lightgreen').grid(row=2,column=2,padx=10,pady=10)
ttk.Label(master=r_frame,textvariable=curr_set['ri'],font="Calibri 16 bold",background="lightgreen").grid(row=2,column=3,padx=10,pady=10)
ttk.Button(master=r_frame,textvariable=btn_text,command=lambda v=user_set['ri']: button(v,keys[4],entry5),style='TButton').grid(row=2,column=4,padx=10,pady=10)

ttk.Label(master=r_frame,text="D",font="Calibri 16 bold",background='lightgreen').grid(row=3,column=0,padx=10,pady=10)
entry6 = ttk.Entry(master=r_frame,textvariable=user_set['rd'])
entry6.grid(row=3,column=1,padx=10,pady=10)
ttk.Label(master=r_frame,text="Current: ",font="Calibri 16 bold",background='lightgreen').grid(row=3,column=2,padx=10,pady=10)
ttk.Label(master=r_frame,textvariable=curr_set['rd'],font="Calibri 16 bold",background="lightgreen").grid(row=3,column=3,padx=10,pady=10)
ttk.Button(master=r_frame,textvariable=btn_text,command=lambda v=user_set['rd']: button(v,keys[5],entry6),style='TButton').grid(row=3,column=4,padx=10,pady=10)




# yaw
s.configure('Frame3.TFrame',background='red')
y_frame = ttk.Frame(master=app,borderwidth=2,relief='raised',style='Frame3.TFrame')
y_frame.grid(row=3,column=0,padx=10,pady=10)

ttk.Label(master=y_frame,text="Yaw",font="Calibri 16 bold",background='red').grid(row=0,column=1,padx=10,pady=10)

ttk.Label(master=y_frame,text="P",font="Calibri 16 bold",background='red').grid(row=1,column=0,padx=10,pady=10)
entry7 = ttk.Entry(master=y_frame,textvariable=user_set['yp'])
entry7.grid(row=1,column=1,padx=10,pady=10)
ttk.Label(master=y_frame,text="Current: ",font="Calibri 16 bold",background='red').grid(row=1,column=2,padx=10,pady=10)
ttk.Label(master=y_frame,textvariable=curr_set['yp'],font="Calibri 16 bold",background="red").grid(row=1,column=3,padx=10,pady=10)
ttk.Button(master=y_frame,textvariable=btn_text,command=lambda v=user_set['yp']: button(v,keys[6],entry7),style='TButton').grid(row=1,column=4,padx=10,pady=10)

ttk.Label(master=y_frame,text="I",font="Calibri 16 bold",background='red').grid(row=2,column=0,padx=10,pady=10)
entry8 = ttk.Entry(master=y_frame,textvariable=user_set['yi'])
entry8.grid(row=2,column=1,padx=10,pady=10)
ttk.Label(master=y_frame,text="Current: ",font="Calibri 16 bold",background='red').grid(row=2,column=2,padx=10,pady=10)
ttk.Label(master=y_frame,textvariable=curr_set['yi'],font="Calibri 16 bold",background="red").grid(row=2,column=3,padx=10,pady=10)
ttk.Button(master=y_frame,textvariable=btn_text,command=lambda v=user_set['yi']: button(v,keys[7],entry8),style='TButton').grid(row=2,column=4,padx=10,pady=10)

ttk.Label(master=y_frame,text="D",font="Calibri 16 bold",background='red').grid(row=3,column=0,padx=10,pady=10)
entry9 = ttk.Entry(master=y_frame,textvariable=user_set['yd'])
entry9.grid(row=3,column=1,padx=10,pady=10)
ttk.Label(master=y_frame,text="Current: ",font="Calibri 16 bold",background='red').grid(row=3,column=2,padx=10,pady=10)
ttk.Label(master=y_frame,textvariable=curr_set['yd'],font="Calibri 16 bold",background="red").grid(row=3,column=3,padx=10,pady=10)
ttk.Button(master=y_frame,textvariable=btn_text,command=lambda v=user_set['yd']: button(v,keys[8],entry9),style='TButton').grid(row=3,column=4,padx=10,pady=10)

# pitch angle
s.configure('Frame4.TFrame',background='orange')
pa_frame = ttk.Frame(master=app,borderwidth=2,relief='raised',style='Frame4.TFrame')
pa_frame.grid(row=1,column=1,padx=10,pady=10)

ttk.Label(master=pa_frame,text="Pitch",font="Calibri 16 bold",background='orange').grid(row=0,column=1,padx=10,pady=10)

ttk.Label(master=pa_frame,text="P",font="Calibri 16 bold",background='orange').grid(row=1,column=0,padx=10,pady=10)
entry10 = ttk.Entry(master=pa_frame,textvariable=user_set['app'])
entry10.grid(row=1,column=1,padx=10,pady=10)
ttk.Label(master=pa_frame,text="Current: ",font="Calibri 16 bold",background='orange').grid(row=1,column=2,padx=10,pady=10)
ttk.Label(master=pa_frame,textvariable=curr_set['app'],font="Calibri 16 bold",background="orange").grid(row=1,column=3,padx=10,pady=10)
ttk.Button(master=pa_frame,textvariable=btn_text,command=lambda v=user_set['app']: button(v,keys[9],entry10),style='TButton').grid(row=1,column=4,padx=10,pady=10)

ttk.Label(master=pa_frame,text="I",font="Calibri 16 bold",background='orange').grid(row=2,column=0,padx=10,pady=10)
entry11 = ttk.Entry(master=pa_frame,textvariable=user_set['api'])
entry11.grid(row=2,column=1,padx=10,pady=10)
ttk.Label(master=pa_frame,text="Current: ",font="Calibri 16 bold",background='orange').grid(row=2,column=2,padx=10,pady=10)
ttk.Label(master=pa_frame,textvariable=curr_set['api'],font="Calibri 16 bold",background="orange").grid(row=2,column=3,padx=10,pady=10)
ttk.Button(master=pa_frame,textvariable=btn_text,command=lambda v=user_set['api']: button(v,keys[10],entry11),style='TButton').grid(row=2,column=4,padx=10,pady=10)

ttk.Label(master=pa_frame,text="D",font="Calibri 16 bold",background='orange').grid(row=3,column=0,padx=10,pady=10)
entry12 = ttk.Entry(master=pa_frame,textvariable=user_set['apd'])
entry12.grid(row=3,column=1,padx=10,pady=10)
ttk.Label(master=pa_frame,text="Current: ",font="Calibri 16 bold",background='orange').grid(row=3,column=2,padx=10,pady=10)
ttk.Label(master=pa_frame,textvariable=curr_set['apd'],font="Calibri 16 bold",background="orange").grid(row=3,column=3,padx=10,pady=10)
ttk.Button(master=pa_frame,textvariable=btn_text,command=lambda v=user_set['apd']: button(v,keys[11],entry12),style='TButton').grid(row=3,column=4,padx=10,pady=10)

# roll angle
s.configure('Frame5.TFrame',background='yellow')
ra_frame = ttk.Frame(master=app,borderwidth=2,relief='raised',style='Frame5.TFrame')
ra_frame.grid(row=2,column=1,padx=10,pady=10)

ttk.Label(master=ra_frame,text="Roll",font="Calibri 16 bold",background='yellow').grid(row=0,column=1,padx=10,pady=10)

ttk.Label(master=ra_frame,text="P",font="Calibri 16 bold",background='yellow').grid(row=1,column=0,padx=10,pady=10)
entry13 = ttk.Entry(master=ra_frame,textvariable=user_set['arp'])
entry13.grid(row=1,column=1,padx=10,pady=10)
ttk.Label(master=ra_frame,text="Current: ",font="Calibri 16 bold",background='yellow').grid(row=1,column=2,padx=10,pady=10)
ttk.Label(master=ra_frame,textvariable=curr_set['arp'],font="Calibri 16 bold",background="yellow").grid(row=1,column=3,padx=10,pady=10)
ttk.Button(master=ra_frame,textvariable=btn_text,command=lambda v=user_set['arp']: button(v,keys[12],entry13),style='TButton').grid(row=1,column=4,padx=10,pady=10)

ttk.Label(master=ra_frame,text="I",font="Calibri 16 bold",background='yellow').grid(row=2,column=0,padx=10,pady=10)
entry14 = ttk.Entry(master=ra_frame,textvariable=user_set['ari'])
entry14.grid(row=2,column=1,padx=10,pady=10)
ttk.Label(master=ra_frame,text="Current: ",font="Calibri 16 bold",background='yellow').grid(row=2,column=2,padx=10,pady=10)
ttk.Label(master=ra_frame,textvariable=curr_set['ari'],font="Calibri 16 bold",background="yellow").grid(row=2,column=3,padx=10,pady=10)
ttk.Button(master=ra_frame,textvariable=btn_text,command=lambda v=user_set['ari']: button(v,keys[13],entry14),style='TButton').grid(row=2,column=4,padx=10,pady=10)

ttk.Label(master=ra_frame,text="D",font="Calibri 16 bold",background='yellow').grid(row=3,column=0,padx=10,pady=10)
entry15 = ttk.Entry(master=ra_frame,textvariable=user_set['ard'])
entry15.grid(row=3,column=1,padx=10,pady=10)
ttk.Label(master=ra_frame,text="Current: ",font="Calibri 16 bold",background='yellow').grid(row=3,column=2,padx=10,pady=10)
ttk.Label(master=ra_frame,textvariable=curr_set['ard'],font="Calibri 16 bold",background="yellow").grid(row=3,column=3,padx=10,pady=10)
ttk.Button(master=ra_frame,textvariable=btn_text,command=lambda v=user_set['ard']: button(v,keys[14],entry15),style='TButton').grid(row=3,column=4,padx=10,pady=10)

# send all button
ttk.Button(master=app,text="Send All",command=send_all,style='TButton').grid(row=0,column=2,padx=10,pady=0)

# quit button
ttk.Button(master=app,text="Quit",command=quit,style='TButton').grid(row=0,column=3,padx=10,pady=0)



# Request gains on startup
threading.Thread(target=wait_for_connect_and_upd, daemon=True).start()

app.mainloop()
print("Server shutdown, goodbye!")