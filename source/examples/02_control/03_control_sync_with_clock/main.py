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
import xdefw.rtt
import physicshelper
import xde.desc.physic.physic_pb2

class MyController(xdefw.rtt.Task):
  
    def __init__(self, taskName, world, robotName):
        task = rtt_interface.PyTaskFactory.CreateTask(taskName)
        xdefw.rtt.Task.__init__(self, task)

        multiBodyModel = xde.desc.physic.physic_pb2.MultiBodyModel()
        mechanism_index = 0
        for m in world.scene.physical_scene.mechanisms:
            if robotName == m.name:
                break
            else:
                mechanism_index = mechanism_index + 1

        multiBodyModel.kinematic_tree.CopyFrom(world.scene.physical_scene.nodes[ mechanism_index ])
        multiBodyModel.meshes.extend(world.library.meshes)
        multiBodyModel.mechanism.CopyFrom(world.scene.physical_scene.mechanisms[ mechanism_index ])
        multiBodyModel.composites.extend(world.scene.physical_scene.collision_scene.meshes)
        self.model = physicshelper.createDynamicModel(multiBodyModel)

        self.tau_out = self.addCreateOutputPort("tau", "VectorXd")
  
    def startHook(self):
        pass
  
    def stopHook(self):
        pass
  
    def updateHook(self):
        self.doUpdate()
  
    def doUpdate(self):
        time.sleep(0.5)
        tau = lgsm.vector([8, 4, 1])
        self.tau_out.write(tau)


#-------------------------------------------------------------------------------
#
# Create Controller
#
#-------------------------------------------------------------------------------

# Create controller
controller = MyController("MyController", world, "p1") # ControllerName, the world instance, RobotName
controller.s.setPeriod(time_step)

# Create clock
import deploy.deployer as ddeployer
clock = xdefw.rtt.Task(ddeployer.load("clock", "dio::Clock", "dio-cpn-clock", ""))
clock.s.setPeriod(0.5)

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
import xdefw.interactive
shell = xdefw.interactive.shell_console()
shell()


