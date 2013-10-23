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

##### Create the kinematic tree

# Each node is a segment.
#
# A node is a tuple (segment_name, segment_mass, H_child_parent, list_of_dofs, list_of_children).
#
# each dof being a tuple (dof_type, center_in_parent, axis_in_parent, damping, qmin, qmax, qref)
# list_of_dofs is a list of degrees of freedom,
# dof_type is either 'hinge' or 'prismatic'. Empty list of dofs means fixed joint.
#
# The last arguments "list_of_children" is a list of nodes.


body  = ["b_root", "b_1", "b_2", "b_3"]
joint = ["j_root", "j_1", "j_2", "j_3"]
mass = 1 #1kg
damp = 2

kin_tree = (body[0], mass, lgsm.Displacement(), [], [
              (body[1], mass, lgsm.Displacementd(.5,0,0,1,0,0,0), [('hinge',[0,0,0],[0,1,0], damp, -3.14, 3.14, 0.2)], [
                (body[2], mass, lgsm.Displacementd(.5,0,0,1,0,0,0), [('hinge',[0,0,0],[0,1,0], damp, -3.14, 3.14, 0.2)], [
                  (body[3], mass, lgsm.Displacementd(.5,0,0,1,0,0,0), [('hinge',[0,0,0],[0,1,0], damp, -3.14, 3.14, 0.2)], []
                )]
              )]
            )])



##### Create a world with description of the kinematic tree
import desc.physic
import desc.graphic

# We create the structure that will contain the description of our robot.
import xde.desc.app.concepts_pb2 as concepts_pb2
robot_data = concepts_pb2.RobotData()

desc.physic.fillKinematicTree(robot_data.physical_robot.multi_body_model.kinematic_tree,
                            tree=kin_tree,
                            fixed_base=True,
                            H_init=lgsm.Displacement())


desc.physic.fillMechanism(robot_data.physical_robot.multi_body_model.mechanism,
                         name="pendulum",
                         root_node="b_root",
                         trim_nodes=[],
                         bodies=body,
                         segments=body)

## We fill the graphical description of the robot.
desc.graphic.fillGraphicalNode(robot_data.graphical_robot.graphical_tree, "robot.root", position=lgsm.Displacement(), scale=[1,1,1])

## We write the description of the robot in a binary file
import desc_core
with open(cpath + "/pendulum.desc", "wb") as f:
  f.write(desc_core.Serialize(robot_data))

##### Fill and deserialize world: register world description in phy & graph agents
import desc.scene
world = desc.scene.createWorld()

import concepts.robot
robot_importer = concepts.robot.RobotImporter("robot_factory", cpath + "/pendulum.desc")
robot_importer.fillLibrary(world)
robot_importer.addInstance(world, "pendulum")

import agents.graphic.builder
import agents.physic.builder

agents.graphic.builder.deserializeWorld(graph, gInterface, world)
agents.physic.builder.deserializeWorld(phy, ms, lmd, world)



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
# Run agents
#
#-------------------------------------------------------------------------------

phy.s.start()
graph.s.start()


##### Interactive shell
import xdefw.interactive
shell = xdefw.interactive.shell_console()
shell()


