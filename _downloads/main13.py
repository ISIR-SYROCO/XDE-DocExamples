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

import iCubcommon

world = iCubcommon.add_iCub(damping=1, H_init=[0,0,1,1,0,0,0], fixed_base=False)
iCubcommon.add_iCub_meshes(world, createComposite=True, compositeOffset=0.001)
iCubcommon.addGround(world)
iCubcommon.addContactLaws(world)
iCubcommon.addCollisionPairs(world)

##### Deserialize world: register world description in phy & graph agents
import agents.graphic.builder
import agents.physic.builder

print "deserializeWorld..."
print "graphic..."
agents.graphic.builder.deserializeWorld(graph, gInterface, world)
print "and physic..."
agents.physic.builder.deserializeWorld(phy, ms, lmd, world)
print "... done."

phy.s.GVM.Robot("iCub").enableGravity(False)


##### Connect physic and graphic agents to see bodies with markers
ocb = phy.s.Connectors.OConnectorBodyStateList.new("ocb", "body_state")

for n in phy.s.GVM.Scene("main").getBodyNames():
    if n != "groundRigidBody":
        ocb.addBody(n)

graph.s.Connectors.IConnectorBody.new("icb", "body_state_H", "mainScene")
graph.getPort("body_state_H").connectTo(phy.getPort("body_state_H"))



#-------------------------------------------------------------------------------
#
# Run agents
#
#-------------------------------------------------------------------------------

phy.s.start()
graph.s.start()


icub = phy.s.GVM.Robot('iCub')
print "To enable gravity, type: icub.enableGravity(True)"

##### Interactive shell
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()




