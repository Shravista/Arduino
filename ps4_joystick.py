#!/usr/bin/env python3

from pyPS4Controller.controller import Controller
import serial
import time
class PS4(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        
        self.linear = 0
        self.angular = 0
        self.MAX  = 2**15
        self.rpi = serial.Serial("/dev/ttyACM0", 9600)
        time.sleep(1)
        
    def on_R2_press(self, value):
        self.linear = value
        try:
            ddir = self.linear//abs(self.linear)
        except Exception as e:
            ddir = 0
        self.linear = abs(self.linear)
        val = self.clip(self.linear)
        val = str(val)
        if ddir < 0:
            msg = "f" + "0"*(3-len(val)) + val
            msg = msg*2 + "\n"
        elif ddir > 0:
            msg = "r" + "0"*(3-len(val)) + val
            msg = msg*2 + "\n" 
        else:
            msg = "s000s000\n"
        print(f"linear = {value,val} Message = ", msg[:-1])
        msg = bytes(msg,"utf-8")
        self.rpi.write(msg)
    
    def on_L3_left(self, value):
        
        msg = b"f180r180\n"
        print("Direction = Left. Message = ", msg[:-1])
        self.rpi.write(msg)
    
    def on_L3_right(self, value):
        
        msg = b"r180f180\n"
        print("Direction = Right. Message = ", msg[:-1])
        self.rpi.write(msg)
    
    def on_L3_x_at_rest(self):
        msg = b"s000s000\n"
        self.rpi.write(msg)
    def on_L3_y_at_rest(self):
        pass   
    def on_L2_press(self, value):
        pass
    
    def on_L3_up(self, value):
        pass
    
    def on_L3_down(self, value):
        pass
        
    def clip(self,val):
        tmp = (val)/(self.MAX)*255 
        return int(tmp)
ps = PS4(interface="/dev/input/js0")
ps.listen(timeout=200)
ps.rpi.close()
