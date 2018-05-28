#!/usr/bin/env python

'''
xvlidarplot.py : A little Python class to display scans from the XV Lidar
             
Copyright (C) 2016 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

#COM_PORT                   = 'COM42'           # Windows
COM_PORT                    = '/dev/ttyACM0'    # Linux

# Arbitrary display params
DISPLAY_CANVAS_SIZE_PIXELS  = 500
DISPLAY_CANVAS_COLOR        = 'black'
DISPLAY_SCAN_LINE_COLOR     = 'yellow'

# XVLIDAR-04LX specs
XVLIDAR_MAX_SCAN_DIST_MM    = 6000
XVLIDAR_DETECTION_DEG       = 360
XVLIDAR_SCAN_SIZE           = 360

from xvlidar import XVLidar

from math import sin, cos, radians
from time import time, sleep, ctime
from sys import exit, version

if version[0] == '3':
    import tkinter as tk
    import _thread as thread
else:
    import Tkinter as tk
    import thread

               
                
class XVLidarPlotter(tk.Frame):
    '''
    XVLidarPlotter extends tk.Frame to plot Lidar scans.
    '''
    
    def __init__(self):
        '''
        Takes no args.  Maybe we could specify colors, lidar params, etc.
        '''
        
        # Create the frame        
        tk.Frame.__init__(self, borderwidth = 4, relief = 'sunken')
        self.master.geometry(str(DISPLAY_CANVAS_SIZE_PIXELS)+ "x" + str(DISPLAY_CANVAS_SIZE_PIXELS))
        self.master.title('XV Lidar [ESC to quit]')
        self.grid()
        self.master.rowconfigure(0, weight = 1)
        self.master.columnconfigure(0, weight = 1)
        self.grid(sticky = tk.W+tk.E+tk.N+tk.S)
        self.background = DISPLAY_CANVAS_COLOR
        
        # Add a canvas for drawing
        self.canvas =  tk.Canvas(self, \
            width = DISPLAY_CANVAS_SIZE_PIXELS, \
            height = DISPLAY_CANVAS_SIZE_PIXELS,\
            background = DISPLAY_CANVAS_COLOR)
        self.canvas.grid(row = 0, column = 0,\
                    rowspan = 1, columnspan = 1,\
                    sticky = tk.W+tk.E+tk.N+tk.S)

        # Set up a key event for exit on ESC
        self.bind('<Key>', self._key)

        # This call gives the frame focus so that it receives input
        self.focus_set()

        # No scanlines initially                             
        self.lines = []
        
        # Connect to the XVLidar
        self.lidar = XVLidar(COM_PORT)
        
        # No scan data to start
        self.scandata = []
        
        # Pre-compute some values useful for plotting
        
        scan_angle_rad = [radians(-XVLIDAR_DETECTION_DEG/2 + (float(k)/XVLIDAR_SCAN_SIZE) * \
                                   XVLIDAR_DETECTION_DEG) for k in range(XVLIDAR_SCAN_SIZE)]
                                   
        self.half_canvas_pix = DISPLAY_CANVAS_SIZE_PIXELS / 2
        scale = self.half_canvas_pix / float(XVLIDAR_MAX_SCAN_DIST_MM)

        self.cos = [-cos(angle) * scale for angle in scan_angle_rad]
        self.sin = [ sin(angle) * scale for angle in scan_angle_rad]
        
        # Add scan lines to canvas, to be modified later        
        self.lines = [self.canvas.create_line(\
                         self.half_canvas_pix, \
                         self.half_canvas_pix, \
                         self.half_canvas_pix + self.sin[k] * XVLIDAR_MAX_SCAN_DIST_MM,\
                         self.half_canvas_pix + self.cos[k] * XVLIDAR_MAX_SCAN_DIST_MM)
                         for k in range(XVLIDAR_SCAN_SIZE)]
                         
        [self.canvas.itemconfig(line, fill=DISPLAY_SCAN_LINE_COLOR) for line in self.lines]
        
        # Start a new thread and set a flag to let it know when we stop running
        thread.start_new_thread( self.grab_scan, () )       
        self.running = True      

    # Runs on its own thread
    def grab_scan(self):

        while True:
            
            # Lidar sends 360 (distance, quality) pairs, which may be empty on start
            self.scandata = [pair[0] for pair in self.lidar.getScan()]

            self.count += 1

            if not self.running:
                break

            # yield to plotting thread
            sleep(.0001)

    def run(self):
        '''
        Call this when you're ready to run.
        '''

        # Record start time and initiate a count of scans for testing
        self.count = 0
        self.start_sec = time()
        self.showcount = 0

        # Start the recursive timer-task
        plotter._task() 

        # Start the GUI
        plotter.mainloop()


    def destroy(self):
        '''
        Called automagically when user clicks X to close window.
        '''  

        self._quit()

    def _quit(self):

        self.running = False
        elapsed_sec = time() - self.start_sec

        del self.lidar

        exit(0)

    def _key(self, event):

        # Make sure the frame is receiving input!
        self.focus_force()
        if event.keysym == 'Escape':
            self._quit()

    def _task(self):

        # Modify the displayed lines according to the current scan
        [self.canvas.coords(self.lines[k], 
            self.half_canvas_pix, \
                    self.half_canvas_pix, \
                    self.half_canvas_pix + self.sin[k] * self.scandata[k],\
                    self.half_canvas_pix + self.cos[k] * self.scandata[k]) \
                    for k in range(len(self.scandata))]    

        # Reschedule this task immediately
        self.after(1, self._task)

        # Record another display for reporting performance
        self.showcount += 1


# Instantiate and pop up the window
if __name__ == '__main__':

    plotter = XVLidarPlotter()

    plotter.run()
