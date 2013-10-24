##### Preambule
import loader
import deploy
deploy.loadTypekitsAndPlugins()

import lgsm

import os, sys
import inspect
cpath = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe()))) + "/"
sys.path.append(cpath)


##### Create physic and graphic agents + pendulum robot
import agents.physic.core
import agents.graphic.simple
import agents.graphic.proto

import desc.scene

import desc.physic
import desc.graphic

import desc.scene
import desc_core
import concepts.robot
import concepts.simple.robot

import xde.desc.app.concepts_pb2 as concepts_pb2

# Physic
def get_physic_agent(time_step):
    phy = agents.physic.core.createAgent("physic", 0)                        # physic agent
    ms = agents.physic.core.createGVMScene(phy, "main", time_step=time_step) # physic scene
    lmd = agents.physic.core.createXCDScene(phy, "xcd", "LMD", lmd_max=.01)  # collision scene

    ms.setGeometricalScene(lmd) # link physic scene and collision scene
    phy.s.setPeriod(time_step)
    
    return phy, ms, lmd


# Graphic
def get_graphic_agent():
    graph = agents.graphic.simple.createAgent("graphic", 0)                          # graphic scne
    gInterface, Nscene, Nwin, Nview = agents.graphic.simple.setupSingleGLView(graph) # graphic interface
    agents.graphic.proto.configureBasicLights(gInterface)
    agents.graphic.proto.configureBasicCamera(gInterface)
    graph.s.Viewer.enableNavigation(True)
    gInterface.SceneryInterface.showGround(True)
    
    return graph, gInterface


# Robot
def create_pendulum(world, p_name="pendulum", p_init=None):
    body  = [p_name+".b_root", p_name+".b_1", p_name+".b_2", p_name+".b_3"]
    joint = [p_name+".j_root", p_name+".j_1", p_name+".j_2", p_name+".j_3"]
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

    # We create the structure that will contain the description of our robot.
    robot_data = concepts_pb2.RobotData()

    desc.physic.fillKinematicTree(robot_data.physical_robot.multi_body_model.kinematic_tree,
                                tree=kin_tree,
                                fixed_base=True,
                                H_init=lgsm.Displacement())


    desc.physic.fillMechanism(robot_data.physical_robot.multi_body_model.mechanism,
                             name=p_name,
                             root_node=body[0],
                             trim_nodes=[],
                             bodies=body,
                             segments=joint)

    ## We fill the graphical description of the robot.
    desc.graphic.fillGraphicalNode(robot_data.graphical_robot.graphical_tree, p_name+".root", position=lgsm.Displacement(), scale=[1,1,1])

    ## We write the description of the robot in a binary file
    with open(cpath + "/"+p_name+".desc", "wb") as f:
      f.write(desc_core.Serialize(robot_data))

    ##### Fill and deserialize world: register world description in phy & graph agents

    robot_importer = concepts.robot.RobotImporter("robot_factory", cpath +"/"+p_name+".desc")
    robot_importer.fillLibrary(world)
    robot_importer.addInstance(world, p_name) 
