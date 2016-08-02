from Tkinter import *
from tkFileDialog import askopenfilename
import Image, ImageTk
import sys
from os import path, getenv
from time import sleep

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


if __name__ == "__main__":
    root = Tk()
    interface = IvyMessagesInterface("WaypointMover")
    #setting up a tkinter canvas with scrollbars
    frame = Frame(root, bd=2, relief=SUNKEN,width=800,height=800)
    frame.grid_rowconfigure(0, weight=1)
    frame.grid_columnconfigure(0, weight=1)
    xscroll = Scrollbar(frame, orient=HORIZONTAL)
    xscroll.grid(row=1, column=0, sticky=E+W)
    yscroll = Scrollbar(frame)
    yscroll.grid(row=0, column=1, sticky=N+S)
    canvas = Canvas(frame, bd=0, xscrollcommand=xscroll.set, yscrollcommand=yscroll.set)
    canvas.grid(row=0, column=0, sticky=N+S+E+W)
    xscroll.config(command=canvas.xview)
    yscroll.config(command=canvas.yview)

    #adding the image
    img = ImageTk.PhotoImage(Image.open("YUV_UV_plane.png"))
    canvas.create_image(0,0,image=img,anchor="nw")
    canvas.config(scrollregion=canvas.bbox(ALL))

    #function to be called when mouse is clicked
    def printcoords(event):
        #outputting x and y coords to console
        print (event.x,event.y)
        msg = PprzMessage("ground", "MOVE_WAYPOINT")
        msg['ac_id']=200
        interface.send(msg)
    #mouseclick event
    canvas.bind("<Button 1>",printcoords)

    frame.pack(fill=BOTH,expand=1)
    root.mainloop()
