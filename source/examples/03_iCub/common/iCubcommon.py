
import numpy as np
import lgsm

import os
import inspect

################################################################################
import desc.graphic
import desc.physic
import desc.core
import desc.scene
import desc.collision

import desc.simple.scene
import desc.simple.physic
import desc.simple.graphic
import desc.simple.collision

import XDEiCub

def add_iCub(damping = 2, H_init=None, fixed_base=True):

    if H_init is None:
        H_init = lgsm.Displacement()
    H_init = lgsm.Displacement(H_init)

    world = desc.scene.createWorld()

    bodies_data = XDEiCub.get_bodies_data()
    kin_tree    = XDEiCub.get_kinematic_tree(damping)

    ########## Build physical scene
    print "add iCub Kinematic Tree"
    desc.physic.fillKinematicTree(world.scene.physical_scene.nodes.add(),
                                tree=kin_tree,
                                fixed_base=fixed_base,
                                H_init=H_init)

    print "add iCub Mechanism"
    desc.physic.addMechanism(world.scene.physical_scene,
                             name="iCub",
                             root_node="waist",
                             trim_nodes=[],
                             bodies=list(bodies_data),
                             segments=list(bodies_data))

    phy_root = desc.physic.findInPhysicalScene(world.scene.physical_scene, "waist")

    def setNodeMomentsOfInertia(node):
        b_name = node.rigid_body.name
        MoI = bodies_data[b_name][1]
        node.rigid_body.moments_of_inertia.extend(MoI)

    desc.core.visitDepthFirst(setNodeMomentsOfInertia, phy_root)

    
    def setNodeMaterial(node):
        node.rigid_body.contact_material = "material.metal"

    desc.core.visitDepthFirst(setNodeMaterial, phy_root)
    
    
    return world



def add_iCub_meshes(world, useCollisionMeshes=False, createComposite=False, compositeOffset=0.001):

    meshes_data = XDEiCub.get_meshes_data()

    current_path = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe())))
    if useCollisionMeshes is False:
        icub_world = desc.scene.parseColladaFile(current_path+"/iCub_meshes.dae")
    else:
        icub_world = desc.scene.parseColladaFile(current_path+"/iCub_collision_meshes.dae")
    
    desc.simple.graphic.addGraphicalTree(world, icub_world, node_name="iCub")
    root_node = desc.core.findInTree(world.scene.graphical_scene.root_node, "iCub")

    for node in root_node.children:
        node.ClearField("position")
        node.position.extend(lgsm.Displacement().tolist())

    
    ########## Build graphical scene
    for body_name in meshes_data:
        body_g_node = desc.graphic.addGraphicalNode(world.scene.graphical_scene, body_name, None, root_node)
        body_g_node.ClearField("position") # Position has to be cleared when adding a new node, otherwise an error related to vector size appears.
        body_g_node.position.extend(lgsm.Displacement().tolist())
        body_g_node.ClearField("scale")    # Scale has to be cleared when adding a new node
        body_g_node.scale.extend([1, 1, 1])

        for sub_mesh, H_body_submesh in meshes_data[body_name]:
            if useCollisionMeshes is False:
                sub_mesh_name = sub_mesh
            else:
                sub_mesh_name = sub_mesh+"_collision"
            sub_mesh_node = desc.core.findInTree(root_node, sub_mesh_name)
            if sub_mesh_node is not None: # if the node is found...
                sub_mesh_node.ClearField("position")
                sub_mesh_node.position.extend(H_body_submesh.tolist())
                
                desc.core.copyTree(body_g_node, sub_mesh_node)
                for i in range(len(root_node.children)):
                    if root_node.children[i].name == sub_mesh_name:
                        del root_node.children[i]
                        break


        # Create composite from submeshes of graphical node
        composite_name = ""
        if createComposite is True:
            if len(body_g_node.children):
                composite_name = body_name+".comp"
                composite = desc.collision.addCompositeMesh(world.scene.physical_scene.collision_scene, composite_name, offset=compositeOffset)
                desc.collision.copyFromGraphicalTree(composite.root_node, body_g_node)
                composite.root_node.ClearField("position")
                composite.root_node.position.extend([0,0,0,1,0,0,0])

        # Create bindings between, physical, graphical and collision scenes
        graph_node      = desc.core.findInTree(world.scene.graphical_scene.root_node, body_name)
        phy_node        = desc.physic.findInPhysicalScene(world.scene.physical_scene, body_name)
        graph_node.name = body_name # it is suitable to have the same name for both graphics and physics.

        phy_node.rigid_body.composite_name=composite_name
    
    
    return world



def addGround(world):
    current_path = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe())))
    ground_world = desc.simple.scene.parseColladaFile(current_path+"/ground.dae")
    phy_ground_world = desc.simple.scene.parseColladaFile(current_path+"/ground_phy_50mm.dae", append_label_library=".phyground")

    desc.simple.graphic.addGraphicalTree(world, ground_world, node_name="ground")
    desc.simple.collision.addCompositeMesh(world, phy_ground_world, composite_name="ground.comp", offset=0.05, clean_meshes=False, ignore_library_conflicts=False)
    desc.simple.physic.addRigidBody(world, "ground", mass=1, contact_material="material.concrete")
    ground_position = lgsm.Displacementd()
    ground_position.setTranslation(lgsm.vector(0,0,-0.1))
    desc.simple.physic.addFixedJoint(world, "ground.joint", "ground", ground_position)

    #Binding graph, phy and coll object
    ground_graph_node      = desc.core.findInTree(world.scene.graphical_scene.root_node, "ground")
    ground_phy_node        = desc.physic.findInPhysicalScene(world.scene.physical_scene, "ground")
    ground_graph_node.name = "ground" # it is suitable to have the same name for both graphics and physics.
    ground_phy_node.rigid_body.composite_name="ground.comp"



def addContactLaws(world, friction_coeff=.4):
    world.scene.physical_scene.contact_materials.extend(["material.concrete", "material.metal"])
    cl = world.scene.physical_scene.contact_laws.add()
    cl.material_i = "material.metal"
    cl.material_j = "material.concrete"
    cl.law = cl.COULOMB
    cl.friction = friction_coeff



def addCollisionPairs(world):
    cp = world.scene.physical_scene.collision_pairs.add()
    cp.body_i = "ground"
    cp.mechanism_j = "iCub"
    cp.queries.add(type=1, enabled=True)

