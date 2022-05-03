#!/usr/bin/env python3
# -*- coding: utf8 -*-
import rospy
from wapabawama_ros.msg import moisture
from time import sleep
import signal
import time
import packer
import sys 
import numpy as np
from SX127x.LoRa import *
from SX127x.board_config import BOARD
from SX127x.LoRaArgumentParser import LoRaArgumentParser
from datetime import datetime
import Jetson.GPIO as GPIO

BOARD.setup()

parser = LoRaArgumentParser("Continous LoRa receiver.")

# python2
try:
    import sys
    reload(sys)
    sys.setdefaultencoding('utf-8')
except:
    pass

# Create Node list
Nodes = 3        # number of Nodes
number_node = 0            # No. node

def quit(signum, frame):
    
    print ('stop fusion')
    sys.exit()
class LoRaGateWay(LoRa):
    def __init__(self, verbose=False):
        rospy.init_node('lora', anonymous=True)
        self.pubb = rospy.Publisher("/moisture", moisture, queue_size=10)   
        super(LoRaGateWay, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0,0,0,0,0,0])    # RX
        self.rate = rospy.Rate(5)

    def on_rx_done(self):
        payload = self.read_payload(nocheck=True)
        data = ''.join([chr(c) for c in payload])
        info = data.split(",")
        moisture_temp = moisture()
#         for i in node_list:
        try:
            if(len(info)==2):  
                # print(info)
                moisture_temp.ID = int(info[0]) 
                moisture_temp.data = float(info[1]) 
                self.pubb.publish(moisture_temp)
        except:
            print("RX error")
        self.pubb.publish(moisture_temp)
        self.set_dio_mapping([0,0,0,0,0,0])    # RX
        self.set_mode(MODE.STDBY)
        sleep(1)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        self.clear_irq_flags(RxDone=1)
        

    def start(self):
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        
            # self.rate.sleep()

        while True:
            sleep(0.5)
            self.set_mode(MODE.RXCONT)
            sys.stdout.flush()
            # if(rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
            #         sys.exit()

lora = LoRaGateWay(verbose=False)
# args = parser.parse_args(lora)
lora.set_mode(MODE.STDBY)
lora.set_pa_config(pa_select=1)
lora.set_freq(434.0)
lora.set_preamble(8)
lora.set_spreading_factor(7)
lora.set_bw(7)
lora.set_coding_rate(1)
lora.set_ocp_trim(100)
try:
    signal.signal(signal.SIGINT, quit)                                
    signal.signal(signal.SIGTERM, quit)
    lora.start()

except KeyboardInterrupt:
    sys.stdout.flush()
    sys.stderr.write("KeyboardInterrupt\n")
finally:
    sys.stdout.flush()
    lora.set_mode(MODE.SLEEP)
    sleep(.5)
    
    BOARD.teardown()
    GPIO.cleanup()
