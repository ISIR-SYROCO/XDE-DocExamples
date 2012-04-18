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

import lgsm

import time

time_step = 0.01

phy, ms, lmd = common.get_physic_agent(time_step)

graph, gInterface = common.get_graphic_agent()

world = common.create_world_and_deserialized(phy, ms, lmd, graph, gInterface)


#-------------------------------------------------------------------------------
#
# Create Connections between phy and graph
#
#-------------------------------------------------------------------------------
##### Connect physic and graphic agents to see bodies with markers
ocb = phy.s.Connectors.OConnectorBodyStateList.new("ocb", "bodyPosition")
graph.s.Connectors.IConnectorFrame.new("icf", "framePosition", "mainScene")
graph.getPort("framePosition").connectTo(phy.getPort("bodyPosition_H"))

# add markers on bodies
for n in phy.s.GVM.Scene("main").getBodyNames():
    ocb.addBody(n)
    gInterface.MarkersInterface.addMarker(n, False)



#-------------------------------------------------------------------------------
#
# Create a new Task. This will be the controller.
#
#-------------------------------------------------------------------------------

import rtt_interface
import dsimi.rtt
import physicshelper

class MyController(dsimi.rtt.Task):
  
    def __init__(self, taskName, world, robotName):
        task = rtt_interface.PyTaskFactory.CreateTask(taskName)
        dsimi.rtt.Task.__init__(self, task)

        self.model = physicshelper.createDynamicModel(world, robotName)

        self.tau_out = self.addCreateOutputPort("tau", "VectorXd")
  
    def startHook(self):
        pass
  
    def stopHook(self):
        pass
  
    def updateHook(self):
        self.doUpdate()
  
    def doUpdate(self):
        time.sleep(0.001)                  # simulate a short time operation
        #time.sleep(0.1)                  # simulate a time-consumming operation
        tau = lgsm.vector([8, 4, 1])
        self.tau_out.write(tau)



#-------------------------------------------------------------------------------
#
# Create Controller
#
#-------------------------------------------------------------------------------

# Create controller
controller = MyController("MyController", world, "p1") # ControllerName, the world instance, RobotName
controller.s.setPeriod(0.001)

##### Create clock, to synchronize phy and controller
import deploy.deployer as ddeployer
clock = dsimi.rtt.Task(ddeployer.load("clock", "dio::Clock", "dio-cpn-clock", "dio/component/"))
clock.s.setPeriod(.01)

# add Input Port in physic agent
phy.s.Connectors.IConnectorRobotJointTorque.new("ict", "p1_", "p1") # ConnectorName, PortName, RobotName
                                                                    # It generates a port named "PortName"+"tau"
phy.addCreateInputPort("clock_trigger", "double")

icps = phy.s.Connectors.IConnectorSynchro.new("icps")
icps.addEvent("p1_tau")
icps.addEvent("clock_trigger")

clock.getPort("ticks").connectTo(phy.getPort("clock_trigger"))
controller.getPort("tau").connectTo(phy.getPort("p1_tau"))

#-------------------------------------------------------------------------------
#
# Run agents
#
#-------------------------------------------------------------------------------

phy.s.start()
graph.s.start()
controller.s.start()
clock.s.start()

##### Interactive shell
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()


