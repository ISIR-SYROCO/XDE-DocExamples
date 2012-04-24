
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

def add_RX90_with_meshes(damping = 2, rx90_offset=0.001):

    H_init = lgsm.Displacement()

    current_path = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe())))
    world = desc.scene.parseColladaFile(current_path + "/rx90.dae")
    root_node = world.scene.graphical_scene.root_node

    for node in root_node.children:
        desc.graphic.setNodeScale(node, [.001, .001, .001])

    nodes_name      = ["node_44", "node_38", "node_32", "node_26", "node_20", "node_14"]
    bodies_name     = ["00", "01", "02", "03", "04", "05"]
    composites_name = ["00.comp", "01.comp", "02.comp", "03.comp", "04.comp", "05.comp"]

    rx90_mass = 0.5
    rx90_damping = damping
    rx90_pos = lgsm.Displacementd(0., 0., 0.0, 0, 0., 0., 1.)
    rx_segments = ["00", "01", "02", "03", "04", "05", "06"]
    rx_bodies = rx_segments

    kin_tree = \
    ("00", rx90_mass, rx90_pos, [], [ # base
      ("01", rx90_mass, lgsm.Displacementd(0, 0, 0.42, 1,0,0, 0), [('hinge', [0,0,0], [0,0,1], rx90_damping, -99*pi, 99*pi, 0.)], [ # elbow
        ("02", rx90_mass, lgsm.Displacementd(0, 0, 0,-0.7071, 0.7071, 0, 0), [('hinge', [0,0,0], [0,1,0], rx90_damping, -99*127.5*pi/180, 99*127.5*pi/180, 0)], [ # arm
          ("03", rx90_mass, lgsm.Displacementd(0, -0.45, 0.049, 0, 0, -0.7071,0.7071) , [('hinge', [0,-0.45,0.049], [0,0,1], rx90_damping, -99*142.5*pi/180, 99*142.5*pi/180, 0.)], [ # forearm
            ("04", rx90_mass, lgsm.Displacementd(0, 0, 0, 0, 0,0, 1) , [('hinge', [0,0,0], [0,0,1], rx90_damping, -99*270*pi/180, 99*270*pi/180, 0.)], [ # wrist
              ("05", rx90_mass, lgsm.Displacementd(0, 0, 0.65, 0.5, 0.5, -0.5, 0.5) , [('hinge', [0,0,0.65], [0,1,0], rx90_damping, -99*122.5*pi/180, 99*122.5*pi/180, 0.)], [ # hand
                ("06", rx90_mass, lgsm.Displacementd(0.085,0, 0, 0.7071,  0, 0.7071, 0) , [('hinge', [0.085,0,0], [1,0,0], rx90_damping, -99*270*pi/180, 99*270*pi/180, 0.)], []) # end effector
              ])
            ])
          ])
        ])
      ])
    ])

    ########## Build physical scene
    phy_root = desc.robot.addKinematicTree(world.scene.physical_scene, parent_node=None, tree=kin_tree, fixed_base=True, H_init=H_init)
    desc.physic.addMechanism(world.scene.physical_scene, name="rx90", root_node="00", trim_nodes=[], bodies=rx_bodies, segments=rx_segments)


    def setNodeMaterial(node):
        node.rigid_body.contact_material = "material.metal"
    desc.core.visitDepthFirst(setNodeMaterial, phy_root)

    for node_name, comp_name in zip(nodes_name, composites_name):
        graph_node = desc.core.findInTree(world.scene.graphical_scene.root_node, node_name)
        composite = desc.collision.addCompositeMesh(world.scene.collision_scene, comp_name, offset=rx90_offset)
        desc.collision.copyFromGraphicalTree(composite.root_node, graph_node)
        composite.root_node.ClearField("position")
        composite.root_node.position.extend([0,0,0,1,0,0,0])

    for node_name, body_name, comp_name in zip(nodes_name, bodies_name, composites_name):
        graph_node = desc.core.findInTree(world.scene.graphical_scene.root_node, node_name)
        graph_node.name = body_name # it is suitable to have the same name for both graphics and physics.
        desc.scene.addBinding(world, body_name, body_name, "", comp_name)



    return world



def addGround(world):
    current_path = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe())))
    ground_world = desc.simple.scene.parseColladaFile(current_path+"/ground.dae")
    phy_ground_world = desc.simple.scene.parseColladaFile(current_path+"/ground_phy_50mm.dae", append_label_library=".phyground")

    desc.simple.graphic.addGraphicalTree(world, ground_world, node_name="ground")
    desc.simple.collision.addCompositeMesh(world, phy_ground_world, composite_name="ground.comp", offset=0.05, clean_meshes=False, ignore_library_conflicts=False)
    desc.simple.physic.addRigidBody(world, "ground", mass=1, contact_material="material.concrete")
    desc.simple.physic.addFixedJoint(world, "ground.joint", "ground", lgsm.Displacementd(0,0,-0.1))
    desc.simple.scene.addBinding(world, phy="ground", graph="ground", graph_ref="", coll="ground.comp")



def addContactLaws(world, friction_coeff=.4):
    world.scene.physical_scene.contact_materials.extend(["material.concrete", "material.metal"])
    cl = world.scene.physical_scene.contact_laws.add()
    cl.material_i = "material.metal"
    cl.material_j = "material.concrete"
    cl.law = cl.COULOMB
    cl.friction = friction_coeff



def addCollisionPairs(world):
    cp = world.scene.collision_pairs.add()
    cp.body_i = "ground"
    cp.mechanism_i = "rx90"
    cp.enabled = True

