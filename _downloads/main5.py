#-------------------------------------------------------------------------------
#
# Load libraries and setup agents
#
#-------------------------------------------------------------------------------
import sys
import os
import inspect
cpath = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe()))) + "/"
sys.path.append(cpath)

import common

import time


#-------------------------------------------------------------------------------
#
# Create a new Task. This will be the controller.
#
#-------------------------------------------------------------------------------

import rtt_interface
import dsimi.rtt

class MyController(dsimi.rtt.Task):

    def __init__(self, Task_name):
        task = rtt_interface.PyTaskFactory.CreateTask(Task_name)
        dsimi.rtt.Task.__init__(self, task)
        
        self.tick = self.addCreateInputPort("controller_tick", "double", True)
        self.tick_ok = False

    def startHook(self):
        pass
  
    def stopHook(self):
        pass
  
    def updateHook(self):
        print "IN  updateHook"
        
        tick_val, self.tick_ok = self.tick.read()
        
        print tick_val, self.tick_ok
        
        if self.tick_ok:
            self.tick_ok = False
            self.doUpdate(tick_val)
  
    def doUpdate(self, tick):
        print "IN doUpdate: tick = "+str(tick)



#-------------------------------------------------------------------------------
#
# Create clock and controller
#
#-------------------------------------------------------------------------------

##### Create controller
controller = MyController("MyController")
controller.s.setPeriod(.01)

##### Create clock, to synchronize phy and controller
import deploy.deployer as ddeployer
clock = dsimi.rtt.Task(ddeployer.load("clock", "dio::Clock", "dio-cpn-clock", "dio/component/"))
clock.s.setPeriod(.1)


##### Connect and synchronize phy and controller
clock.getPort("ticks").connectTo(controller.getPort("controller_tick"))


#-------------------------------------------------------------------------------
#
# Run agents
#
#-------------------------------------------------------------------------------

controller.s.start()
clock.s.start()

time.sleep(5)

controller.s.stop()
clock.s.stop()

##### Interactive shell
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()




