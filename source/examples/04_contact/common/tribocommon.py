
from numpy import pi
import lgsm

import os
import inspect

################################################################################
import desc.graphic
import desc.physic
import desc.core
import desc.scene
import desc.robot
import desc.collision

import desc.simple.scene
import desc.simple.physic
import desc.simple.graphic
import desc.simple.collision

def add_tribo_with_meshes(damping = 2, offset=0.02):

    current_path = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe())))
    world = desc.scene.parseColladaFile(current_path + "/tribo_meshes.dae")
    root_node = world.scene.graphical_scene.root_node

    for node in root_node.children:
        desc.graphic.setNodeScale(node, [1,1,1])


    ##### Build tribologic arm
    bodies_name = ["Base", "Arm", "Forearm", "Hand", "Finger"]

    mass = 1

    kin_tree = \
    ("Base", mass, lgsm.Displacement(), [], [ # base
      ("Arm", mass, lgsm.Displacementd(0,0,.5,1,0,0,0), [('hinge', [0,0,0], [0,1,0], damping, -2*pi, 2*pi, -.3)], [
        ("Forearm", mass, lgsm.Displacementd(0,0,1,1,0,0,0), [('hinge', [0,0,.5], [0,1,0], damping, -2*pi, 2*pi, -.3)], [
          ("Hand", mass, lgsm.Displacementd(0,0,1,1,0,0,0), [('hinge', [0,0,.5], [0,1,0], damping, -2*pi, 2*pi, -.3)], [
            ("Finger", mass, lgsm.Displacementd(0,0,1,1,0,0,0), [('hinge', [0,0,.5], [0,1,0], damping, -2*pi, 2*pi, -.3)], [
            ])
          ])
        ])
      ])
    ])
    H_init = lgsm.Displacement(-1,1,.15)
    phy_root = desc.robot.addKinematicTree(world.scene.physical_scene, parent_node=None, tree=kin_tree, fixed_base=True, H_init=H_init)
    desc.physic.addMechanism(world.scene.physical_scene, name="tribo", root_node="Base", trim_nodes=[], bodies=bodies_name, segments=bodies_name)

    def setNodeMaterial(node):
        node.rigid_body.contact_material = "material.metal"
    desc.core.visitDepthFirst(setNodeMaterial, phy_root)

    #### Build obstacle
    desc.simple.physic.addRigidBody(world, "Obstacle", mass=1, contact_material="material.gum")
    desc.simple.physic.addFixedJoint(world, "Obstacle.joint", "Obstacle", lgsm.Displacement(2., 1, .5))
    #desc.simple.physic.addFreeJoint(world, "Obstacle.joint", "Obstacle", lgsm.Displacement(2., 1, 2.))
  
    
    ##### Add meshes: create composite
    for node_name in bodies_name[1:] + ["Obstacle"]:
        graph_node = desc.core.findInTree(world.scene.graphical_scene.root_node, node_name)
        composite = desc.collision.addCompositeMesh(world.scene.collision_scene, node_name+".comp", offset=offset)
        desc.collision.copyFromGraphicalTree(composite.root_node, graph_node)
        composite.root_node.ClearField("position")
        composite.root_node.position.extend([0,0,0,1,0,0,0])
        composite.root_node.ClearField("scale")
        composite.root_node.scale.extend([1,1,1])

        graph_node.name = node_name # it is suitable to have the same name for both graphics and physics.
        desc.scene.addBinding(world, node_name, node_name, "", node_name+".comp")

    ##### Interactive shell
    # import dsimi.interactive
    # shell = dsimi.interactive.shell()
    # shell()
    
    
    
    return world



def addGround(world):
    current_path = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe())))
    ground_world = desc.simple.scene.parseColladaFile(current_path+"/ground.dae")
    phy_ground_world = desc.simple.scene.parseColladaFile(current_path+"/ground_phy_50mm.dae", append_label_library=".phyground")

    desc.simple.graphic.addGraphicalTree(world, ground_world, node_name="ground")
    desc.simple.collision.addCompositeMesh(world, phy_ground_world, composite_name="ground.comp", offset=0.05, clean_meshes=False, ignore_library_conflicts=False)
    desc.simple.physic.addRigidBody(world, "ground", mass=1, contact_material="material.concrete")
    desc.simple.physic.addFixedJoint(world, "ground.joint", "ground", lgsm.Displacementd(0,0,-0.1)) #lgsm.Displacementd(1,0,-0.6, .95,0,.05,0))
    desc.simple.scene.addBinding(world, phy="ground", graph="ground", graph_ref="", coll="ground.comp")



def addContactLaws(world, friction_coeff=.4):
    world.scene.physical_scene.contact_materials.extend(["material.concrete", "material.metal", "material.gum"])
    cl = world.scene.physical_scene.contact_laws.add()
    cl.material_i = "material.metal"
    cl.material_j = "material.concrete"
    cl.law = cl.COULOMB
    cl.friction = .4
    
    cl = world.scene.physical_scene.contact_laws.add()
    cl.material_i = "material.gum"
    cl.material_j = "material.concrete"
    cl.law = cl.COULOMB
    cl.friction = 1.5
    
    cl = world.scene.physical_scene.contact_laws.add()
    cl.material_i = "material.gum"
    cl.material_j = "material.metal"
    cl.law = cl.COULOMB
    cl.friction = .7



def addCollisionPairs(world):
    cp = world.scene.collision_pairs.add()
    cp.body_i = "ground"
    cp.mechanism_i = "tribo"
    cp.enabled = True
    
    cp = world.scene.collision_pairs.add()
    cp.body_i = "ground"
    cp.body_j = "Obstacle"
    cp.enabled = True
    
    cp = world.scene.collision_pairs.add()
    cp.body_i = "Obstacle"
    cp.mechanism_j = "tribo"
    cp.enabled = True

