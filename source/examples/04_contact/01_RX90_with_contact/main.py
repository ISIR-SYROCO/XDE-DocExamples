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
cpath = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe()))) + "/../common"
sys.path.append(cpath)

import common

import lgsm
import time

phy, ms, lmd = common.get_physic_agent()

graph, gInterface = common.get_graphic_agent()




#-------------------------------------------------------------------------------
#
# Create a new Task. This will be the Kinematic controller.
#
#-------------------------------------------------------------------------------
import rtt_interface
import dsimi.rtt
import physicshelper

class KinematicController(dsimi.rtt.Task):
  
    def __init__(self, taskName, world, robotName):
        task = rtt_interface.PyTaskFactory.CreateTask(taskName)
        dsimi.rtt.Task.__init__(self, task)

        # model instance
        self.model = physicshelper.createDynamicModel(world, robotName)

        # create input ports
        self.c_in = self.addCreateInputPort("cont", "SMsg", True)
        self.c_ok = False
        self.c = None
        
        self.tau_out = self.addCreateOutputPort("tau", "VectorXd")


    def startHook(self):
        pass
  
    def stopHook(self):
        pass
  
    def updateHook(self):
        if not self.c_ok:
            self.c,self.c_ok = self.c_in.read()
        
        if self.c_ok:
            self.c_ok = False
            self.doUpdate(self.c)

  
    def doUpdate(self, c):
        model = self.model
        
        if len(c.cpt) == 0:
            print "no contact yet"
        else:
            # That what we get 
            print "Nb of contact: ", len(c.cpt)
            print "for contact 0: "
            cc = c.cpt[0]
            print "    ai :", cc.ai[:]
            print "    aj :", cc.aj[:]
            print "    gap:", cc.gap
            print "    ni :", cc.ni[:]
            print "    nj :", cc.nj[:]
            print "    normalForce:", cc.normalForce
            time.sleep(.1)

        tau = lgsm.zeros(model.nbInternalDofs())
        self.tau_out.write(tau)
        
#-------------------------------------------------------------------------------
#
# Create the robot with the desc module.
#
# The goal is to make a general description of the robot.
# The robot is then create in the physical scene with
# the deserializeWorld method.
#
#-------------------------------------------------------------------------------

import RX90common

world = RX90common.add_RX90_with_meshes()
RX90common.addGround(world)
RX90common.addContactLaws(world)
RX90common.addCollisionPairs(world)


##### Deserialize world: register world description in phy & graph agents
import agents.graphic.builder
import agents.physic.builder

print "deserializeWorld..."
print "graphic..."
agents.graphic.builder.deserializeWorld(graph, gInterface, world)
print "and physic..."
agents.physic.builder.deserializeWorld(phy, ms, lmd, world)
print "done."




#-------------------------------------------------------------------------------
#
# Connect Physic and graphic agent
#
#-------------------------------------------------------------------------------
##### Connect physic and graphic agents to see bodies with markers
ocb = phy.s.Connectors.OConnectorBodyStateList.new("ocb", "body_state")

graph.s.Connectors.IConnectorFrame.new("icf", "framePosition", "mainScene")
graph.getPort("framePosition").connectTo(phy.getPort("body_state_H"))

# add markers on bodies
for b in world.scene.rigid_body_bindings:
    if len(b.graph_node) and len(b.rigid_body):
        ocb.addBody(str(b.rigid_body))

graph.s.Connectors.IConnectorBody.new("icb", "body_state_H", "mainScene")
graph.getPort("body_state_H").connectTo(phy.getPort("body_state_H"))



#-------------------------------------------------------------------------------
#
# Create Kinematic controller & clock
#
#-------------------------------------------------------------------------------
controller = KinematicController("MyController", world, "rx90") # ControllerName, the world instance, RobotName
controller.s.setPeriod(0.01)

phy.s.Connectors.IConnectorRobotJointTorque.new("icjt", "rx90_", "rx90")

##### Create clock
import deploy.deployer as ddeployer
clock = dsimi.rtt.Task(ddeployer.load("clock", "dio::Clock", "dio-cpn-clock", "dio/component/"))
clock.s.setPeriod(0.01) #clock period == phy period


phy.addCreateInputPort("clock_trigger", "double")

icps = phy.s.Connectors.IConnectorSynchro.new("icps")
icps.addEvent("rx90_tau")
icps.addEvent("clock_trigger")

clock.getPort("ticks").connectTo(phy.getPort("clock_trigger"))

controller.getPort("tau").connectTo(phy.getPort("rx90_tau"))

#-------------------------------------------------------------------------------
#
# Connection for information about contacts and visualization
#
#-------------------------------------------------------------------------------
occ = phy.s.Connectors.OConnectorContactBody.new("occ", "contacts")
occ.addInteraction(phy.s.GVM.Robot("rx90").getSegmentRigidBody2("03"), "ground")
occ.addInteraction(phy.s.GVM.Robot("rx90").getSegmentRigidBody2("04"), "ground")
occ.addInteraction(phy.s.GVM.Robot("rx90").getSegmentRigidBody2("05"), "ground")
occ.addInteraction(phy.s.GVM.Robot("rx90").getSegmentRigidBody2("06"), "ground")

icc = graph.s.Connectors.IConnectorContacts.new("icc", "contacts", "mainScene")
icc.setMaxProximity(.05)
icc.setGlyphScale(2)

graph.getPort("contacts").connectTo(phy.getPort("contacts"))


controller.getPort("cont").connectTo(phy.getPort("contacts"))

#-------------------------------------------------------------------------------
#
# Run agents
#
#-------------------------------------------------------------------------------

phy.s.start()
graph.s.start()
controller.s.start()
clock.s.start()

phy.s.agent.triggerUpdate()


rx90 = phy.s.GVM.Robot('rx90')
rx90.enableGravity(True)
print "To enable gravity, type: rx90.enableGravity(True)"

##### Interactive shell
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()

# on obtient:
# ai
# aj
# gap
# ni
# nj
# normalForce



