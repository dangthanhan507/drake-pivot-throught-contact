'''
    Load bubble.sdf with drake and simulate it with pydrake
'''
from pydrake.geometry import StartMeshcat
from pydrake.multibody.plant import MultibodyPlant, MultibodyPlantConfig, AddMultibodyPlant
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from pydrake.visualization import AddDefaultVisualization
from manipulation.scenarios import AddMultibodyTriad
from pydrake.multibody.parsing import Parser
from pydrake.all import Quaternion
import numpy as np

if __name__ == '__main__':
    config = MultibodyPlantConfig()
    config.time_step = 1e-3
    config.penetration_allowance = 1e-3
    config.contact_model = "hydroelastic_with_fallback"
    config.contact_surface_representation = "polygon"
    config.discrete_contact_approximation = "tamsi"
    
    meshcat = StartMeshcat()
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(config, builder)
    plant: MultibodyPlant = plant
    
    parser = Parser(plant, scene_graph)
    
    
    # model = parser.AddModelsFromUrl("package://drake_models/wsg_50_description/sdf/schunk_wsg_50_hydro_bubble.sdf")[0]
    # model = parser.AddModelsFromUrl("package://drake_models/wsg_50_description/sdf/schunk_wsg_50_deformable_bubble.sdf")[0] # does not specify deformable, must be added with drake code
    
    model = parser.AddModels("wsg_50_description/sdf/schunk_wsg_50_hydro_bubble.sdf")[0]
    
    plant.Finalize()
    
    AddDefaultVisualization(builder, meshcat)
    
    diagram = builder.Build()
    
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    
    meshcat.StartRecording()
    diagram.ForcedPublish(context)
    context.SetTime(0.0)
    
    #print number of plant positions
    # plant.SetPositions(plant_context, plant.GetModelInstanceByName("Schunk_Gripper"), np.zeros(3))
    plant.SetFreeBodyPose(plant_context, plant.GetBodyByName("gripper"), RigidTransform(RotationMatrix.MakeZRotation(np.pi/2), [0, 0, 0.5]))
    
    diagram.ForcedPublish(context)
    meshcat.StopRecording()
    meshcat.PublishRecording()
    input()