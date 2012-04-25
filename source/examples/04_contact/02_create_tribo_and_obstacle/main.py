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


phy, ms, lmd      = common.get_physic_agent()
graph, gInterface = common.get_graphic_agent()
clock             = common.get_clock_agent()



#-------------------------------------------------------------------------------
#
# Create robot
#
#-------------------------------------------------------------------------------
import tribocommon

world = tribocommon.add_tribo_with_meshes()
tribocommon.addGround(world)
tribocommon.addContactLaws(world)
tribocommon.addCollisionPairs(world)

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
# Create Controllers
#
#-------------------------------------------------------------------------------

import Ctrlcommon
controller = Ctrlcommon.ContactController("myCtrl", world, "tribo")



#-------------------------------------------------------------------------------
#
# Common connections between agents
#
#-------------------------------------------------------------------------------
common.connnectBodyState(phy, graph, world)

contactPairs = [("tribo", "ground"), ("tribo", "Obstacle"), ("ground", "Obstacle")]
common.connnectContactVisualization(phy, graph, contactPairs)


#-------------------------------------------------------------------------------
#
# Custom connections
#
#-------------------------------------------------------------------------------
controller.getPort("cont").connectTo(phy.getPort("contacts"))


#-------------------------------------------------------------------------------
#
# Synchronization
#
#-------------------------------------------------------------------------------
phy.addCreateInputPort("clock_trigger", "double")
phy.s.Connectors.IConnectorRobotJointTorque.new("icjt", "tribo_", "tribo")

icps = phy.s.Connectors.IConnectorSynchro.new("icps")
icps.addEvent("tribo_tau")
icps.addEvent("clock_trigger")

clock.getPort("ticks").connectTo(phy.getPort("clock_trigger"))
controller.getPort("tau").connectTo(phy.getPort("tribo_tau"))




#-------------------------------------------------------------------------------
#
# Run agents
#
#-------------------------------------------------------------------------------
phy.s.start()
graph.s.start()
clock.s.start()
controller.s.start()

phy.s.agent.triggerUpdate()

# tribo = phy.s.GVM.Robot('tribo')
# tribo.enableGravity(True)
# print "To enable gravity, type: tribo.enableGravity(True)"


while controller.c is None:
    print "no contact yet"
while len(controller.c.cpt) == 0:
    print "no contact yet"

phy.s.stop()
clock.s.stop()
controller.s.stop()


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



