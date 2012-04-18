################################################################################
#
# Preambule
#
################################################################################
from arboris.core import World
from arboris.robots import icub
from arboris.massmatrix import principalframe, transport
from arboris.homogeneousmatrix import inv as Hinv
from arboris.homogeneousmatrix import transl, roty, rotzyx, rotx
import numpy
numpy.set_printoptions(suppress=True)
from numpy import dot, eye, pi



################################################################################
#
# Create world with iCub
#
################################################################################
w = World()
icub.add(w)
w.update_dynamic()

jLim = icub.get_joint_limits(in_radian=True)
tauLim = icub.get_torque_limits()

# interesting values we want to obtain
H_body_com = {}
body_mass  = {}
body_moment_of_inertia = {}
H_parentCoM_bCoM = {}

list_of_dof = {}
list_of_children = {}

# We compute the displacement from the body frame to the body center of mass, aligned with the principal axis of inertia
H_body_com["ground"] = eye(4)
for b in w.getbodies()[1:]:
    H_body_com[b.name] = principalframe(b.mass)
    Mb_com = transport(b.mass, H_body_com[b.name]) # if all good, Mb_com is diagonal
    body_mass[b.name] = Mb_com[5,5]
    body_moment_of_inertia[b.name] = tuple(Mb_com[[0,1,2], [0,1,2]])


# We compute the displacement between the centers of mass:
# H_parentCoM_bCoM = H_parentCoM_parent * H_parent_b * H_b_bCoM  
#                <=> (H_0_parent * H_parent_parentCoM)^(-1) * (H_0_b * H_b_bCoM)
for b in w.getbodies()[1:]:
    parent = b.parentjoint.frame0.body
    H_parentCoM_bCoM[b.name] = dot( Hinv(  dot(parent.pose, H_body_com[parent.name]) ), dot(b.pose, H_body_com[b.name]) )





for b in w.getbodies()[1:]:
    j = b.parentjoint
    
    list_of_children[b.name] = [childj.frame1.body.name for childj in b.childrenjoints]
    
    if j.name in icub.get_joints_data():
        parent = j.frame0.body
        child  = j.frame1.body
        
        # We look for the vector and direction between body center of Mass and the hinge center and axis of rotation
        # H_parentCoM_hinge = (H_parent_parentCoM)^(-1) * H_parent_hinge
        # V_parentCoM_hinge = H_parentCoM_hinge[0:3,3]
        H_parent_hinge = j.frame0.bpose
        H_parentCoM_hinge = dot( Hinv(H_body_com[parent.name]), H_parent_hinge)
        V_parentCoM_hinge = tuple(H_parentCoM_hinge[0:3,3])
        hinge_axis_in_parentCoM = tuple(H_parentCoM_hinge[0:3,2])
        
        list_of_dof[b.name] = (V_parentCoM_hinge, hinge_axis_in_parentCoM, jLim[j.name][0], jLim[j.name][1])




################################################################################
#
# This part is defined with respect to the blender model of iCub.
# It defines the bodies meshes, their submeshes and their positions.
#
################################################################################
def get_meshes_data():
    """
    """

    return {
    'waist'     : ["waist_mesh"],
    'lap_belt_1': ["pelvis_mesh"],
    'lap_belt_2': [],
    'chest':      ["torso_mesh", "chest_cylinder_mesh"],

    'neck_1': ["neck_1_mesh"],
    'neck_2': ["neck_2_mesh"],
    'head':   ["head_mesh", "head_sphere_mesh"],

    'l_shoulder_1': ["l_shoulder_1_mesh"],
    'l_shoulder_2': ["l_shoulder_2_mesh"],
    'l_arm':        ["l_upperarm_mesh", "l_arm_cyl_mesh"],
    'l_elbow_1':    ["l_elbow_1_mesh"],
    'l_forearm':    ["l_forearm_mesh", "l_forearm_cyl_mesh"],
    'l_wrist_1':    ["l_wrist_1_mesh"],
    'l_hand':       ["l_hand_mesh"],

    'r_shoulder_1': ["r_shoulder_1_mesh"],
    'r_shoulder_2': ["r_shoulder_2_mesh"],
    'r_arm':        ["r_upperarm_mesh", "r_arm_cyl_mesh"],
    'r_elbow_1':    ["r_elbow_1_mesh"],
    'r_forearm':    ["r_forearm_mesh", "r_forearm_cyl_mesh"],
    'r_wrist_1':    ["r_wrist_1_mesh"],
    'r_hand':       ["r_hand_mesh"],

    'l_hip_1':   ["l_hip_1_mesh"],
    'l_hip_2':   ["l_hip_2_mesh"],
    'l_thigh':   ["l_upperleg_mesh", "l_thigh_cyl1_mesh", "l_thigh_cyl2_mesh"],
    'l_shank':   ["l_lowerleg_mesh", "l_shank_cyl_mesh"],
    'l_ankle_1': ["l_ankle_1_mesh"],
    'l_foot':    ["l_foot_mesh", "l_foot_cyl_mesh"],

    'r_hip_1':   ["r_hip_1_mesh"],
    'r_hip_2':   ["r_hip_2_mesh"],
    'r_thigh':   ["r_upperleg_mesh", "r_thigh_cyl1_mesh", "r_thigh_cyl2_mesh"],
    'r_shank':   ["r_lowerleg_mesh", "r_shank_cyl_mesh"],
    'r_ankle_1': ["r_ankle_1_mesh"],
    'r_foot':    ["r_foot_mesh", "r_foot_cyl_mesh"],
    }

def get_meshes_positions():
    return {
        "head_mesh"      :     transl(0,0,-0.055),
        "torso_mesh"     : dot(transl(0,0.024,-0.144), roty(pi)),
        "pelvis_mesh"    : dot(transl(0.031,0.04,0), rotzyx(pi, -pi/2., 0)),

        "l_upperleg_mesh":     transl(0,0,-.05),
        "r_upperleg_mesh": dot(transl(0,0, .05), roty(pi)),
        "l_lowerleg_mesh": rotzyx(0,pi/2.,-pi/2.),
        "r_lowerleg_mesh": rotzyx(0,pi/2.,-pi/2.),
        "l_foot_mesh"    : dot(transl(-0.041,0,-0.03), rotzyx(-pi/2., 0,-pi/2.)),
        "r_foot_mesh"    : dot(transl(-0.041,0, 0.03), rotzyx( pi/2., 0, pi/2.)),

        "l_upperarm_mesh": dot(transl(0,0.01, 0.14), rotx(pi/2.)),
        "r_upperarm_mesh": dot(transl(0,0.01,-0.14), rotx(pi/2.)),
        "l_forearm_mesh" : dot(transl(0,0,-0.004), rotzyx( pi/2.,-pi/2.,0)),
        "r_forearm_mesh" : dot(transl(0,0, 0.004), rotzyx(-pi/2.,-pi/2.,0)),
        
        ## simple meshes
        "waist_mesh"     : transl(.006, 0,-.1),
        "chest_cylinder_mesh": transl(0, 0, -.0447),
        "neck_1_mesh"    : eye(4),
        "neck_2_mesh"    : eye(4),
        "head_sphere_mesh": transl(0, 0, .0825),
        
        "l_shoulder_1_mesh": transl(0, 0, .07224),
        "l_shoulder_2_mesh": eye(4),
        "l_arm_cyl_mesh": transl(0, 0, .078),
        "l_elbow_1_mesh": eye(4),
        "l_forearm_cyl_mesh": transl(0, 0, .07),
        "l_wrist_1_mesh": eye(4),
        "l_hand_mesh": transl(.0345, 0, 0),

        "r_shoulder_1_mesh": transl(0, 0, -.07224),
        "r_shoulder_2_mesh": eye(4),
        "r_arm_cyl_mesh": transl(0, 0, -.078),
        "r_elbow_1_mesh": eye(4),
        "r_forearm_cyl_mesh":transl(0, 0, -.07),
        "r_wrist_1_mesh": eye(4),
        "r_hand_mesh": transl(-.0345, 0, 0),
        
        "l_hip_1_mesh": transl(0, 0, .0375),
        "l_hip_2_mesh": eye(4),
        "l_thigh_cyl1_mesh": transl(0, 0, -.112),
        "l_thigh_cyl2_mesh": dot(transl(0, 0, -.224), roty(pi/2)),
        "l_shank_cyl_mesh": dot(transl(0, -.1065, 0), rotx(pi/2)),
        "l_ankle_1_mesh": eye(4),
        "l_foot_cyl_mesh": transl(0,0,.0125),

        "r_hip_1_mesh": transl(0, 0, -.0375),
        "r_hip_2_mesh": eye(4),
        "r_thigh_cyl1_mesh": transl(0, 0, .112),
        "r_thigh_cyl2_mesh": dot(transl(0, 0, .224), roty(pi/2)),
        "r_shank_cyl_mesh": dot(transl(0, -.1065, 0), rotx(pi/2)),
        "r_ankle_1_mesh": eye(4),
        "r_foot_cyl_mesh": transl(0,0,-.0125),
         }

################################################################################
#
# Create XDEiCub.py from arboris-python data
#
################################################################################
import loader
import lgsm

modfile = """
\"\"\"
This file has been automatically generated to convert iCub model
from arboris-python into XDE.
\"\"\"

import lgsm
"""

def RL(myList, tol=10):
    return [round(v, tol) for v in myList] if hasattr(myList, "__len__") else round(myList, tol)


#####
modfile += """
def get_bodies_data():
    \"\"\" Get bodies mass, inertia and displacement from parent frame.
        
    Returns a dictionnary {'body name': (mass, moments_of_inertia, H_parent_body)}.
    \"\"\"
    return {
"""
for b in body_mass:
    Disp_parent_body = lgsm.Displacement(lgsm.vector(H_parentCoM_bCoM[b][0:3,3]), lgsm.Quaternion(H_parentCoM_bCoM[b][0:3,0:3]))
    modfile += "            '{0}': ({1}, {2}, lgsm.Displacement({3})),\n".format(b, RL(body_mass[b]),
                                                              RL(body_moment_of_inertia[b]),
                                                              RL(Disp_parent_body.tolist()))#H_parentCoM_bCoM[b])

modfile += "            }\n\n"


#####
modfile += """
def get_joints_data():
    \"\"\" Get joints data.
           
    Returns a dictionnary {'joints name': ('parent body name',
                                           'child body name',
                                           Hinge_center_in_parent_frame,
                                           Hinge_axis_in_parent_frame,
                                           q_min,
                                           q_max,
                                           tau_max)}.
    \"\"\"
    return {
"""
for j, val in icub.get_joints_data().items():
    modfile += "            '{0}': ('{1}', '{2}', {3}, {4}, {5}, {6}, {7}),\n".format(j, val[0], val[2],
                                                                       RL(list_of_dof[val[2]][0]), RL(list_of_dof[val[2]][1]),
                                                                       RL(list_of_dof[val[2]][2]), RL(list_of_dof[val[2]][3]),
                                                                       RL(tauLim[j]))

modfile += "            }\n\n"



#####
modfile += """
def get_meshes_data():
    \"\"\" Get meshes data.
           
    Returns a dictionnary {'body name': [('submesh1 name', H_body_submesh1), ...]}.
    \"\"\"
    return {
"""
mesh_positions = get_meshes_positions()
for b, val in get_meshes_data().items():
    modfile += "            '{0}': [\n".format(b)
    for sub_mesh in val:
        H_b_sm = dot(Hinv(H_body_com[b]), mesh_positions[sub_mesh])
        Disp_body_submesh = lgsm.Displacement(lgsm.vector(H_b_sm[0:3,3]), lgsm.Quaternion(H_b_sm[0:3,0:3]))
        modfile += "                      ('{0}', lgsm.Displacement({1})),\n".format(sub_mesh, RL(Disp_body_submesh.tolist()))
    modfile += "                         ],\n"

modfile += "            }\n\n"




#####
modfile += """
def get_kinematic_tree(joint_damping=1.):
    \"\"\" Get kinematic tree as defined in the desc module of XDE.
    \"\"\"

    bodies_data = get_bodies_data()
    joints_data = get_joints_data()
    
    nodes = {}
    for b, val  in bodies_data.items():
        mass, inertias, H_parent_body = val
        nodes[b] = [b, mass, H_parent_body, [], []]
    
    for j, val in joints_data.items():
        p_name, c_name, V_p_hinge, hinge_axis_in_p, qmin, qmax, tau_max = val
        nodes[c_name][3].append(['hinge', V_p_hinge, hinge_axis_in_p, joint_damping, qmin, qmax, 0])
        nodes[p_name][4].append(nodes[c_name])
    
    return nodes['waist']

"""


#####
f= open("XDEiCub.py", "w")
f.write(modfile)
f.close()