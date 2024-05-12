from pydrake.geometry import StartMeshcat
from pydrake.multibody.plant import MultibodyPlant, MultibodyPlantConfig, AddMultibodyPlant
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from pydrake.visualization import AddDefaultVisualization
from manipulation.scenarios import AddMultibodyTriad
from pydrake.multibody.parsing import Parser
from pydrake.all import ConstantVectorSource, LeafSystem, Quaternion, SpatialVelocity
import numpy as np

from shape_lib import AddGround, AddSphere, AddBox, MakeControllableXYZ

def decompose_freebody(object_state: np.ndarray):
    obj_quaternion = object_state[:4]
    obj_pos = object_state[4:7]
    obj_rotdot = object_state[7:10]
    obj_v = object_state[10:13]
    return obj_quaternion, obj_pos, obj_rotdot, obj_v

def decompose_manipulator_params(M: np.ndarray, C: np.ndarray, tau_g: np.ndarray):
    # M = [3,3], C = [3]
    M_finger = M[:3,:3]
    M_object = M[3:,3:]
    
    C_finger = C[:3]
    C_object = C[3:]
    
    tau_g_finger = tau_g[:3]
    tau_g_object = tau_g[3:]
    return M_finger, M_object, C_finger, C_object, tau_g_finger, tau_g_object

class PivotController(LeafSystem):
    def __init__(self, plant: MultibodyPlant):
        LeafSystem.__init__(self)
        
        self.plant_context_ = plant.CreateDefaultContext()
        self.DeclareVectorInputPort("finger_state", 6)
        self.DeclareVectorInputPort("object_state", 13)
        self.DeclareVectorOutputPort("force", 3, self.CalcForce)
    def CalcForce(self, context, output):
        state = self.EvalVectorInput(context, 0).get_value()
        object_state = self.EvalVectorInput(context, 1).get_value()
        plant.SetPositions(self.plant_context_, finger_instance, state[:3])
        plant.SetVelocities(self.plant_context_, finger_instance, state[3:])
        
        obj_quat, obj_pos, obj_rotdot, obj_v = decompose_freebody(object_state)
        obj_quat = obj_quat / np.linalg.norm(obj_quat)
        obj_quat = Quaternion(obj_quat)
        plant.SetFreeBodyPose(self.plant_context_, plant.GetBodyByName("object_body"), RigidTransform(obj_quat, obj_pos))
        plant.SetFreeBodySpatialVelocity(plant.GetBodyByName("object_body"), SpatialVelocity(obj_rotdot, obj_v), self.plant_context_)
        # calc Mass Matrix and Coriolis from context
        M = plant.CalcMassMatrixViaInverseDynamics(plant_context)
        C = plant.CalcBiasTerm(plant_context)
        tau_g = plant.CalcGravityGeneralizedForces(plant_context)
        M_finger, M_object, C_finger, C_object, tau_g_finger, tau_g_object = decompose_manipulator_params(M, C, tau_g)
        
        R_finger2world = RollPitchYaw(0.0,0.0,0.0).ToRotationMatrix().matrix()
        R_obj2world = RotationMatrix(obj_quat).matrix()
        R_finger2obj = R_obj2world.T @ R_finger2world
        
        # calculate forces in finger frame
        # z_force is for
        z_force = -2.0
        
        # PID for controlling y rotation on object
        ry = RotationMatrix(obj_quat).ToRollPitchYaw().pitch_angle()
        target_ry = np.pi/6
        Kp = 1.0
        Kd = 0.1
        x_force = Kp * (target_ry - ry) + Kd * (0 - obj_rotdot[1])
        
        force = np.array([x_force, 0, z_force])
        force = R_obj2world @ force # convert to world force which finger is in
        
        output.SetFromVector(force)
        pass

if __name__ == '__main__':
    config = MultibodyPlantConfig()
    config.time_step = 1e-3
    config.penetration_allowance = 1e-3
    config.contact_model = "hydroelastic_with_fallback"
    config.contact_surface_representation = "triangle"
    config.discrete_contact_approximation = "tamsi"
    
    meshcat = StartMeshcat()
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(config, builder)
    plant: MultibodyPlant = plant
    parser = Parser(plant, scene_graph)
    
    # Add Ground, Add Cube Object, Add actuated sphere finger
    # Cube is rigid, Sphere is rigid, Ground is hydroelastic
    
    cube_width = 0.5
    sphere_radius = 0.05
    platform_height = 0.1
    
    platform_mu = 2.0
    box_mu = 2.0
    finger_mu = 1.0
    
    box_mass = 1.0
    finger_mass = 0.1
    
    platform_instance = AddBox(plant,"platform", lwh=(5.0,5.0,platform_height), mass=box_mass, mu = platform_mu, color=[0,0,1,0.5], hydro=True)
    box_instance = AddBox(plant, "object", lwh=(cube_width,cube_width,cube_width), mass=finger_mass, mu = box_mu, color=[1,0,0,0.5], hydro=False)
    finger_instance = AddSphere(plant, "finger", radius=sphere_radius, mass=0.1, mu = finger_mu, color=[0,1,0,0.5], hydro=False)
    
    box_friction_coeff = 2*platform_mu*box_mu / (platform_mu + box_mu)
    finger_friction_coeff = 2*box_mu*finger_mu / (box_mu + finger_mu)
    
    # add prismatic joints on finger
    MakeControllableXYZ(plant, finger_instance, "finger_body")
    
    AddGround(plant)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("platform_body", platform_instance), RigidTransform(RollPitchYaw(0.0,0.0,0.0), np.array([0,0,platform_height/2])))
    plant.Finalize()
    
    AddDefaultVisualization(builder, meshcat)
    
    # zero_input = builder.AddSystem(ConstantVectorSource([0, 0, 0]))
    controller = builder.AddSystem(PivotController(plant))
    builder.Connect(plant.get_state_output_port(finger_instance), controller.get_input_port(0))
    builder.Connect(plant.get_state_output_port(box_instance), controller.get_input_port(1))
    builder.Connect(controller.get_output_port(), plant.get_actuation_input_port(finger_instance))
    
    diagram = builder.Build()
    
    simulator = Simulator(diagram)
    plant_context = plant.GetMyMutableContextFromRoot(simulator.get_mutable_context())
    
    plant.SetFreeBodyPose(plant_context, plant.GetBodyByName("object_body"), RigidTransform(RotationMatrix.MakeZRotation(0), [-cube_width/2, 0, cube_width/2 + platform_height]))
    plant.SetPositions(plant_context, finger_instance, np.array([0.0, 0.0, cube_width + sphere_radius + platform_height]))
    diagram.ForcedPublish(simulator.get_context())
    
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    meshcat.StartRecording()
    simulator.AdvanceTo(1.0)
    meshcat.StopRecording()
    meshcat.PublishRecording()
    input()