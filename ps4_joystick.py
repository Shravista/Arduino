#!/usr/bin/env python3

from pyPS4Controller.controller import Controller

class PS4(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

ps = PS4(interface="/dev/input/js0")
ps.listen()
