

##### Preambule
import loader
import deploy
deploy.loadTypekitsAndPlugins()

import numpy as np
import lgsm


##### Create physic and graphic agents
import agents.physic.core
import agents.graphic.simple
import agents.graphic.proto

time_step=.01

# Physic
def get_physic_agent():
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




################################################################################
import desc.graphic as myDescGraphic
import desc.physic as myDescPhysic
import desc.core as myDescCore
import desc.scene as myDescScene
import desc.robot as myDescRobot

import XDEiCub

def add_iCub():
    world = myDescScene.parseColladaFile("iCub_meshes.dae")

    bodies_data = XDEiCub.get_bodies_data()
    meshes_data = XDEiCub.get_meshes_data()

########## Build physical scene
    damping = 2
    kin_tree = XDEiCub.get_kinematic_tree(damping)
    
    print "add iCub Kinematic Tree"
    phy_root = myDescRobot.addKinematicTree(world.scene.physical_scene,
                                parent_node=None,
                                tree=kin_tree,
                                fixed_base=True,
                                H_init=lgsm.Displacement(0.5,0,0,1,0,0,0))


    print "add iCub Mechanism"
    myDescPhysic.addMechanism(world.scene.physical_scene,
                             name="iCub",
                             root_node="waist",
                             trim_nodes=[],
                             bodies=list(bodies_data),
                             segments=list(bodies_data))


    def setNodeMomentOfInertia(node):
        b_name = node.rigid_body.name
        MoI = bodies_data[b_name][1]
        node.rigid_body.moments_of_inertia.extend(MoI)
                
    myDescCore.visitDepthFirst(setNodeMomentOfInertia, phy_root)


########## Build graphical scene
    root_node = world.scene.graphical_scene.root_node
    
    def setNodePosition(node, H):
        node.ClearField("position")
        node.position.extend(H.tolist())
    map(lambda node: setNodePosition(node, lgsm.Displacement()), root_node.children)
    
    for body_name in meshes_data:
        body_g_node = myDescGraphic.addGraphicalNode(world.scene.graphical_scene, body_name, None, root_node)
        myDescGraphic.setNodeScale(body_g_node, [1,1,1])

        for sub_mesh, H_body_submesh in meshes_data[body_name]:
            sub_mesh_node = myDescCore.findInTree(root_node, sub_mesh)
            sub_mesh_node.ClearField("position")
            sub_mesh_node.position.extend(H_body_submesh.tolist())
            
            myDescCore.copyTree(body_g_node, sub_mesh_node)
            for i in range(len(root_node.children)):
                if root_node.children[i].name == sub_mesh:
                    del root_node.children[i]
                    break

        myDescScene.addBinding(world, body_name, body_name, "", "")

    
    return world

