from pydrake.geometry import StartMeshcat
from pydrake.multibody.plant import MultibodyPlant, MultibodyPlantConfig, AddMultibodyPlant
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from pydrake.visualization import AddDefaultVisualization
from manipulation.scenarios import AddMultibodyTriad
from pydrake.multibody.parsing import Parser
from pydrake.all import Quaternion, ConstantVectorSource
import numpy as np


from shape_lib import AddGround, AddSphere, AddBox, MakeControllableXYZ

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
    platform_instance = AddBox(plant,"platform", lwh=(5.0,5.0,platform_height), mass=1.0, mu = 1.0, color=[0,0,1,0.5], hydro=True)
    box_instance = AddBox(plant, "object", lwh=(cube_width,cube_width,cube_width), mass=1.0, mu = 1.0, color=[1,0,0,0.5], hydro=False)
    finger_instance = AddSphere(plant, "finger", radius=sphere_radius, mass=1.0, mu = 1, color=[0,1,0,0.5], hydro=False)
    
    # add prismatic joints on finger
    MakeControllableXYZ(plant, finger_instance, "finger_body")
    
    AddGround(plant)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("platform_body", platform_instance), RigidTransform(RollPitchYaw(0.0,0.0,0.0), np.array([0,0,platform_height/2])))
    plant.Finalize()
    
    AddDefaultVisualization(builder, meshcat)
    
    zero_input = builder.AddSystem(ConstantVectorSource([0, 0, 0]))
    builder.Connect(zero_input.get_output_port(), plant.get_actuation_input_port(finger_instance))
    
    diagram = builder.Build()
    
    simulator = Simulator(diagram)
    plant_context = plant.GetMyMutableContextFromRoot(simulator.get_mutable_context())
    
    plant.SetFreeBodyPose(plant_context, plant.GetBodyByName("object_body"), RigidTransform(RotationMatrix.MakeZRotation(0), [0, 0, cube_width/2 + platform_height]))
    plant.SetPositions(plant_context, finger_instance, np.array([0.0, 0.0, cube_width + sphere_radius + platform_height]))
    diagram.ForcedPublish(simulator.get_context())
    
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    meshcat.StartRecording()
    simulator.AdvanceTo(1.0)
    meshcat.StopRecording()
    meshcat.PublishRecording()
    input()