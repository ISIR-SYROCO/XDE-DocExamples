

##### Preambule
import loader
import deploy
deploy.loadTypekitsAndPlugins()

import numpy as np
import lgsm


##### Create physic and graphic agents + pendulum robot
import agents.physic.core
import agents.graphic.simple
import agents.graphic.proto

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


import desc.scene
import desc.robot


# Robot
def create_pendulum(world, p_name="pendulum", p_init=None):

    body  = [p_name+"_"+n for n in ["b_root", "b_1", "b_2", "b_3"]]
    mass = 1 #1kg
    damp = 2

    if p_init is None:
        p_init = lgsm.Displacement()

    kin_tree = (body[0], mass, p_init, [], [
                  (body[1], mass, lgsm.Displacementd(.5,0,0,1,0,0,0), [('hinge',[0,0,0],[0,1,0], damp, -3.14, 3.14, 0.2)], [
                    (body[2], mass, lgsm.Displacementd(.5,0,0,1,0,0,0), [('hinge',[0,0,0],[0,1,0], damp, -3.14, 3.14, 0.2)], [
                      (body[3], mass, lgsm.Displacementd(.5,0,0,1,0,0,0), [('hinge',[0,0,0],[0,1,0], damp, -3.14, 3.14, 0.2)], []
                    )]
                  )]
                )])


    # Create a description of the kinematic tree
    desc.robot.addKinematicTree(world.scene.physical_scene,
                                parent_node=None,
                                tree=kin_tree,
                                fixed_base=True,
                                H_init=lgsm.Displacement())

    desc.physic.addMechanism(world.scene.physical_scene,
                             name=p_name,
                             root_node=body[0],
                             trim_nodes=[],
                             bodies=body,
                             segments=body)


import desc.scene
import agents.graphic.builder
import agents.physic.builder

def create_world_and_deserialized(phy, ms, lmd, graph, gInterface):

    world = desc.scene.createWorld()

    # add some pendulums
    create_pendulum(world, "p1", lgsm.Displacement())
    create_pendulum(world, "p2", lgsm.Displacement(0,.5,0,1,0,0,0))
    create_pendulum(world, "p3", lgsm.Displacement(0,1.,0,1,0,0,0))

    ##### Deserialize world: register world description in phy & graph agents
    agents.graphic.builder.deserializeWorld(graph, gInterface, world)
    agents.physic.builder.deserializeWorld(phy, ms, lmd, world)
    
    ##### Configure some robots
    phy.s.GVM.Robot("p2").enableGravity(False)
    
    return world





