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

print sys.path

import common

import lgsm

phy, ms, lmd = common.get_physic_agent()

graph, gInterface = common.get_graphic_agent()



#-------------------------------------------------------------------------------
#
# Create the robot with the desc module.
#
# The goal is to make a general description of the robot.
# The robot is then create in the physical scene with
# the deserializeWorld method.
#
#-------------------------------------------------------------------------------

world = common.add_iCub()


##### Deserialize world: register world description in phy & graph agents
import agents.graphic.builder
import agents.physic.builder

print "deserializeWorld..."
agents.graphic.builder.deserializeWorld(graph, gInterface, world)
agents.physic.builder.deserializeWorld(phy, ms, lmd, world)
print "... done."


phy.s.GVM.Robot("iCub").enableGravity(True)


##### Connect physic and graphic agents to see bodies with markers
ocb = phy.s.Connectors.OConnectorBodyStateList.new("ocb", "body_state")
graph.s.Connectors.IConnectorFrame.new("icf", "framePosition", "mainScene")
graph.getPort("framePosition").connectTo(phy.getPort("body_state_H"))

# add markers on bodies
for n in phy.s.GVM.Scene("main").getBodyNames():
    if n != "groundRigidBody":
        ocb.addBody(n)
        gInterface.MarkersInterface.addMarker(n, False)

graph.s.Connectors.IConnectorBody.new("icb", "body_state_H", "mainScene")

graph.getPort("body_state_H").connectTo(phy.getPort("body_state_H"))



#-------------------------------------------------------------------------------
#
# Run agents
#
#-------------------------------------------------------------------------------

phy.s.start()
graph.s.start()


##### Interactive shell
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()




