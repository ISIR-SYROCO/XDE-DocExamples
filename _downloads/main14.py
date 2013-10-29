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

import RX90common

world = RX90common.add_RX90_with_meshes()


##### Deserialize world: register world description in phy & graph agents
import agents.graphic.builder
import agents.physic.builder

print "deserializeWorld..."
print "graphic..."
agents.graphic.builder.deserializeWorld(graph, gInterface, world)
print "and physic..."
agents.physic.builder.deserializeWorld(phy, ms, lmd, world)
print "done."




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
# Run agents
#
#-------------------------------------------------------------------------------

phy.s.start()
graph.s.start()

rx90 = phy.s.GVM.Robot('RX90')
rx90.enableGravity(False)
print "To enable gravity, type: rx90.enableGravity(True)"

##### Interactive shell
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()




