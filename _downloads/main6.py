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

time_step = 0.01

phy, ms, lmd = common.get_physic_agent(time_step)

graph, gInterface = common.get_graphic_agent()


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
        tau = lgsm.vector([8, 4, 1])
        self.tau_out.write(tau)




#-------------------------------------------------------------------------------
#
# Create the robot with the desc module.
#
#-------------------------------------------------------------------------------

import desc.scene

world = desc.scene.createWorld()

# add some pendulums
common.create_pendulum(world, "p1", lgsm.Displacement())
common.create_pendulum(world, "p2", lgsm.Displacement(0,.5,0,1,0,0,0))
common.create_pendulum(world, "p3", lgsm.Displacement(0,1.,0,1,0,0,0))

##### Deserialize world: register world description in phy & graph agents
import agents.graphic.builder
import agents.physic.builder

agents.graphic.builder.deserializeWorld(graph, gInterface, world)
agents.physic.builder.deserializeWorld(phy, ms, lmd, world)


controller = MyController("MyController", world, "p1")
controller.s.setPeriod(time_step)


##### Connect physic and graphic agents to see bodies with markers
ocb = phy.s.Connectors.OConnectorBodyStateList.new("ocb", "bodyPosition")
graph.s.Connectors.IConnectorFrame.new("icf", "framePosition", "mainScene")
graph.getPort("framePosition").connectTo(phy.getPort("bodyPosition_H"))

# add markers on bodies
for n in phy.s.GVM.Scene("main").getBodyNames():
    ocb.addBody(n)
    gInterface.MarkersInterface.addMarker(n, False)


##### Configure some robots
phy.s.GVM.Robot("p2").enableGravity(False)

##### Connect Controller and robots
phy.s.Connectors.IConnectorRobotJointTorque.new("ict", "p1_", "p1") # ConnectorName, PortName, RobotName
                                                                    # It generates a port named "PortName"+"tau"
controller.getPort("tau").connectTo(phy.getPort("p1_tau"))

#-------------------------------------------------------------------------------
#
# Run agents
#
#-------------------------------------------------------------------------------

phy.s.start()
graph.s.start()
controller.s.start()

##### Interactive shell
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()

