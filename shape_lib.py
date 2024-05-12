#===============================================
# shape_lib.py
# 
# File for working with geometric primitives
# in drake. 
#===============================================

from pydrake.multibody.plant import MultibodyPlant
from pydrake.geometry import Cylinder, HalfSpace, Box, ProximityProperties, Sphere
from pydrake.multibody.tree import RigidBody, SpatialInertia, UnitInertia
from pydrake.math import RigidTransform
from pydrake.multibody.tree import PrismaticJoint, RevoluteJoint
import numpy as np
from manipulation.scenarios import AddMultibodyTriad
from pydrake.multibody.tree import WeldJoint, FixedOffsetFrame

from contact_lib import AddContactModel



def RegisterVisualShape(plant: MultibodyPlant, name: str, body: RigidBody,
                        shape, color=[1,0,0,1]):
    plant.RegisterVisualGeometry(
        body, RigidTransform(), shape, name, color
    )

def RegisterShape(plant: MultibodyPlant, name:str, body: RigidBody, 
                  shape, prop: ProximityProperties, color=[1,0,0,1], rt=RigidTransform()):
    
    if plant.geometry_source_is_registered():
        plant.RegisterCollisionGeometry(
            body, rt, shape, name, prop
        )
        plant.RegisterVisualGeometry(
            body, rt, shape, name, color
        )
        
def AddGround(plant: MultibodyPlant):
    ground_color = [0.5, 1.0, 0.5, 1.0]
    ground_prop = AddContactModel(plant, halfspace_slab=0.5, hydro_mod = 1e6, dissip=0.0, mu_static=1.0, res_hint=0.01)
    RegisterShape(plant, "GroundVisualGeometry", plant.world_body(), HalfSpace(), ground_prop, ground_color)
    
    
def AddBox(plant: MultibodyPlant, name: str, lwh=(1.0,1.0,1.0), mass=1.0, mu = 1.0, color=[1,0,0,1], hydro=True):
    box_instance = plant.AddModelInstance(name)

    
    box_body = plant.AddRigidBody(f"{name}_body",
                box_instance,
                SpatialInertia(mass=mass,
                               p_PScm_E=np.array([0.0,0.0,0.0]),
                               G_SP_E=UnitInertia.SolidBox(*lwh)
                               )
                )
    box = Box(*lwh)
    
    dissip = 0
    # hydro_mod = 5e4
    hydro_mod = 1e6

    if hydro:
        box_props = AddContactModel(plant, mu_static=mu, hydro_mod= hydro_mod, dissip = dissip, res_hint=0.01)
    else:
        box_props = AddContactModel(plant, mu_static=mu, res_hint=0.01)
    RegisterShape(plant, name, box_body, box, box_props, color)
    return box_instance

def AddSphere(plant: MultibodyPlant, name: str, radius=1.0, mass=1.0, mu = 1, color=[1,0,0,1], hydro=True):
    sphere_instance = plant.AddModelInstance(name)
    
    sphere_body = plant.AddRigidBody(f"{name}_body",
                sphere_instance,
                SpatialInertia(mass=mass,
                               p_PScm_E=np.array([0.0,0.0,0.0]),
                               G_SP_E=UnitInertia.SolidSphere(radius)
                               )
                )
    sphere = Sphere(radius)
    
    dissip = 0
    hydro_mod = 1e6
    
    if hydro:
        sphere_props = AddContactModel(plant, mu_static=mu, hydro_mod= hydro_mod, dissip = dissip, res_hint=0.01)
    else:
        sphere_props = AddContactModel(plant, mu_static=mu, res_hint=0.01)
    RegisterShape(plant, name, sphere_body, sphere, sphere_props, color)
    return sphere_instance


def MakeControllableXYZ(plant: MultibodyPlant, instance, name: str):
    false_body1 = plant.AddRigidBody(
        f"{name}_false1",
        instance,
        SpatialInertia(0, [0, 0, 0], UnitInertia(0, 0, 0)),
    )
    false_body2 = plant.AddRigidBody(
        f"{name}_false2",
        instance,
        SpatialInertia(0, [0, 0, 0], UnitInertia(0, 0, 0)),
    )

    u_x = plant.AddJoint(
        PrismaticJoint(
            f"{name}_x",
            plant.world_frame(),
            plant.GetFrameByName(f"{name}_false1"),
            [1,0,0],
            -10.0,
            10.0
        )
    )
    u_y = plant.AddJoint(
        PrismaticJoint(
            f"{name}_y",
            plant.GetFrameByName(f"{name}_false1"),
            plant.GetFrameByName(f"{name}_false2"),
            [0,1,0],
            -10.0,
            10.0
        )
    )
    u_z = plant.AddJoint(
        PrismaticJoint(
            f"{name}_z",
            plant.GetFrameByName(f"{name}_false2"),
            plant.GetFrameByName(f"{name}"),
            [0,0,1],
            -10.0,
            10.0
        )
    )

    plant.AddJointActuator(f"{name}_x", u_x)
    plant.AddJointActuator(f"{name}_y", u_y)
    plant.AddJointActuator(f"{name}_z", u_z)