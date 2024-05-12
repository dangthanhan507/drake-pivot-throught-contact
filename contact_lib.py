#===============================================
# contact_lib.py
# 
# File for working with hydroelastic contact
# 
#===============================================
from pydrake.geometry import (
    AddContactMaterial, 
    ProximityProperties, 
    AddCompliantHydroelasticPropertiesForHalfSpace, 
    AddCompliantHydroelasticProperties, 
    AddRigidHydroelasticProperties,
    Meshcat
)

from pydrake.multibody.plant import ContactModel, ContactResults, HydroelasticContactInfo
from pydrake.geometry import MeshcatVisualizer, StartMeshcat, TriangleSurfaceMesh, TriangleSurfaceMeshFieldLinear
from pydrake.multibody.meshcat import ContactVisualizer, ContactVisualizerParams
from pydrake.geometry import TriangleSurfaceMesh, SurfaceTriangle


from pydrake.systems.framework import LeafSystem, DiagramBuilder
from pydrake.common.value import Value
from pydrake.multibody.plant import ContactResults
from pydrake.multibody.plant import MultibodyPlant, CoulombFriction
from pydrake.multibody.meshcat import ContactVisualizer, ContactVisualizerParams
import numpy as np
import open3d as o3d
from collections import defaultdict

def AddContactModel(plant: MultibodyPlant, halfspace_slab=0.0, mu_static=0.0, **kwargs):
    '''
    Paramters:
    ==========
    @param plant
    @param half_space_slab
    @param mu_static

    @param kwargs (dictionary)
    ===============================================================================
    | Compliant Parameters                                                        |
    ===============================================================================
    | Key       | Value
    -------------------------------------------------------------------------------
    | hydro_mod   | - measure of stiffness of material (pressure over penetration). 
    |             | - float (Pa (N/m^2))
    -------------------------------------------------------------------------------
    | dissip      | - energy dampening property of contact on object.
    |             | - >0 float (s/m) recommended dissip \in [0,3] and default: 1
    -------------------------------------------------------------------------------
    | res_hint    | - controls fineness of meshes from shapes. 
    |             | - coarse (fast but not accurate), fine (slow but accurate)
    |             | - float (meters)
    -------------------------------------------------------------------------------
    | mu_static   | - coefficient of static friction
    |             | - >0 float.
    -------------------------------------------------------------------------------
    | mu_dynamic  | - coefficient of dynamic friction
    |             | - >0 float recommended to keep same as mu_static
    -------------------------------------------------------------------------------
    ===============================================================================
    | Rigid Parameters                                                            |
    ===============================================================================
    | Key       | Value
    -------------------------------------------------------------------------------
    | res_hint    | - controls fineness of meshes from shapes. 
    |             | - coarse (fast but not accurate), fine (slow but accurate)
    |             | - float (meters)
    -------------------------------------------------------------------------------
    | mu_static   | - coefficient of static friction
    |             | - >0 float. Doesn't just apply to hydroelastic
    -------------------------------------------------------------------------------
    | mu_dynamic  | - coefficient of dynamic friction
    |             | - >0 float recommended to keep same as mu_static
    -------------------------------------------------------------------------------
    '''
    prop = ProximityProperties()
    contact_type = "compliant" if "hydro_mod" in kwargs else "rigid"
    mu_dynamic = (kwargs["mu_dynamic"] if "mu_dynamic" in kwargs else mu_static)
    res_hint   = (kwargs["res_hint"] if "res_hint" in kwargs else 1.0)
    dissip     = (kwargs["dissip"] if "dissip" in kwargs else 1.0)
    AddContactMaterial(dissip, None, CoulombFriction(mu_static, mu_dynamic), prop)

    if contact_type == "rigid":
        AddRigidHydroelasticProperties(res_hint, prop)        

    elif contact_type == "compliant":
        hydro_mod = kwargs["hydro_mod"]
        if halfspace_slab == 0.0:
            AddCompliantHydroelasticProperties(res_hint, hydro_mod, prop)
        else:
            AddCompliantHydroelasticPropertiesForHalfSpace(halfspace_slab, hydro_mod, prop)

    return prop


class ContactForceReporter(LeafSystem):
    def __init__(self,period=0.1, offset=0.0):
        LeafSystem.__init__(self)
        self.DeclareAbstractInputPort(name='contact_results',
                                     model_value=Value(ContactResults()))
        
        self.DeclarePeriodicPublishEvent(period_sec=period, offset_sec=0.0, publish=self.Publish)

        self.wrench_hash = defaultdict(list)
        self.offset = offset
        self.ts = []
    def Publish(self, context):
        if context.get_time() > self.offset:
            contact_results = self.get_input_port().Eval(context)
            
            num_hydroelastic_contacts = contact_results.num_hydroelastic_contacts()
            if num_hydroelastic_contacts > 0:
                self.ts.append(context.get_time())
            for c in range(num_hydroelastic_contacts):
                hydroelastic_contact_info = contact_results.hydroelastic_contact_info(c)
                
                spatial_force = hydroelastic_contact_info.F_Ac_W()
                force = spatial_force.translational().reshape((3,1))
                torque = spatial_force.rotational().reshape((3,1))
                wrench = np.vstack((torque,force))

                self.wrench_hash[c].append(wrench)

class ContactReporter(LeafSystem):
    '''
        Drake LeafSystem reporting contact results.

        You get contact results from plant:
        ---------------------------------------
        plant.get_contact_results_output_port()

        taken straight from tutorial...

    '''
    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclareAbstractInputPort(name='contact_results',
                                     model_value=Value(ContactResults()))
        
        self.DeclarePerStepPublishEvent(self.Publish)
    def Publish(self, context):
        print()
        print(f'ContactReporter::Publish() called at time {context.get_time()}')
        contact_results = self.get_input_port().Eval(context)
        
        num_hydroelastic_contacts = contact_results.num_hydroelastic_contacts()
        print(f'num_hydroelastic_contacts() = {num_hydroelastic_contacts}')
        
        for c in range(num_hydroelastic_contacts):
            print()
            print(f'hydroelastic_contact_info({c}): {c}-th hydroelastic contact patch')
            hydroelastic_contact_info = contact_results.hydroelastic_contact_info(c)
            
            spatial_force = hydroelastic_contact_info.F_Ac_W()
            print('F_Ac_W(): spatial force (on body A, at centroid of contact surface, in World Frame) = ')
            print(f'{spatial_force}')
            
            print('contact_surface()')
            contact_surface = hydroelastic_contact_info.contact_surface()
            num_faces = contact_surface.num_faces()
            total_area = contact_surface.total_area()
            centroid = contact_surface.centroid()
            print(f"total_area(): area of contact surface in m^2 = {total_area}")
            print(f"num_faces(): number of polygons or triangles = {num_faces}")
            print(f"centroid(): centroid (in World frame) = {centroid}")

            if contact_surface.HasGradE_M():
                print(f"EvaluateGradE_M_W(0): gradient of face 0 (em) = {contact_surface.EvaluateGradE_M_W(0)}")

            # M and N in drake documentation refers to two objects.
            # if N is rigid there is no eval gradient of N.
            if contact_surface.HasGradE_N():
                print(f"EvaluateGradE_M_W(0): gradient of face 0 (en) = {contact_surface.EvaluateGradE_N_W(0)}")

            '''
            mesh representation of surface and numbers associated w/ mesh
            '''
            print()
            print("Getting Mesh Pressure Info")
            print("===========================")
            if contact_surface.is_triangle():
                
                mesh = contact_surface.tri_mesh_W()
                meshfield = contact_surface.tri_e_MN()


                num_tri = mesh.num_triangles()
                normal0 = mesh.face_normal(0)

                print(mesh.triangles()[0].vertex(0))
                print(type(mesh.vertices()))
                input()

                print('\ttri_mesh_W()')
                print(f'\tnum_triangles(): number of triangles = {num_tri}')
                print(f'\tface_normal(0): normal of 0-th triangle = {normal0}')

                # print(meshfield.EvaluateGradient(0))
                print(f'\tvertex pressure 0 = {meshfield.EvaluateAtVertex(0)}')
                print(f'\teval pressure 0 at (0,0,0) = {meshfield.Evaluate(0,np.array([0,1,0]))}')

            else:
                mesh = contact_surface.poly_mesh_W()
                meshfield = contact_surface.poly_e_MN()
            


def add_contact_viz(builder: DiagramBuilder, plant: MultibodyPlant, meshcat: Meshcat):
    ContactVisualizer.AddToBuilder(
        builder, plant, meshcat,
        ContactVisualizerParams(
            publish_period= 1.0 / 256.0,
            newtons_per_meter= 2e1,
            newton_meters_per_meter= 1e-1))
    

class ContactOpen3D(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclareAbstractInputPort(name="contact_results",model_value=Value(ContactResults()))
        self.DeclarePerStepPublishEvent(self.Publish)

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.vis.get_render_option().mesh_show_wireframe = True

        self.open3d_mesh = None

    def Publish(self, context):
        contact_results = self.get_input_port().Eval(context)

        if self.open3d_mesh is not None:
            self.vis.remove_geometry(self.open3d_mesh, False)

        for contact_idx in range(contact_results.num_hydroelastic_contacts()):
            hydroelastic_contact_info: HydroelasticContactInfo = contact_results.hydroelastic_contact_info(contact_idx)
            spatial_force = hydroelastic_contact_info.F_Ac_W()
            contact_surface = hydroelastic_contact_info.contact_surface()


            #hydroelastic mesh info
            contact_surface_mesh: TriangleSurfaceMesh = contact_surface.tri_mesh_W()
            pressure_field: TriangleSurfaceMeshFieldLinear = contact_surface.tri_e_MN()
            pressure_field_elements = [pressure_field.EvaluateAtVertex(i) for i in
                                       range(contact_surface_mesh.num_vertices())]
            


            vertices = np.array(contact_surface_mesh.vertices())
            organized_vertices = np.array([[triangle.vertex(i) for i in range(3)] for triangle in contact_surface_mesh.triangles()])

            # Build pressure arrows.
            start_points = []
            end_points = []
            pressure_scale = 1e6
            for tri_idx in range(contact_surface_mesh.num_triangles()):
                center_point = contact_surface_mesh.CalcCartesianFromBarycentric(tri_idx, np.array([1. / 3, 1. / 3, 1. / 3]))
                start_points.append(center_point)
                pressure_on_face = np.mean([pressure_field_elements[v] for v in
                            [contact_surface_mesh.element(tri_idx).vertex(i) for i in range(3)]]) / pressure_scale
                face_normal = contact_surface_mesh.face_normal(tri_idx)
                end_points.append(center_point + (pressure_on_face * face_normal))

            start_points = np.array(start_points)
            end_points = np.array(end_points)
            # print(tri_indices)

            #using vedo option
            # contact_mesh_vedo = Mesh([vertices, organized_vertices], c="red", alpha=0.5)
            # contact_mesh_vedo.wireframe(True)
            # vedo_plt = Plotter(shape=(1, 1))
            # vedo_plt.at(0).add(contact_mesh_vedo)
            # vedo_plt.interactive().close()

            #construct open3d accepting
            self.open3d_mesh = o3d.geometry.TriangleMesh()
            self.open3d_mesh.vertices = o3d.utility.Vector3dVector(vertices)
            self.open3d_mesh.triangles = o3d.utility.Vector3iVector(organized_vertices)
            self.open3d_mesh.compute_vertex_normals()



            # o3d.visualization.draw_geometries([self.open3d_mesh], mesh_show_back_face=True, mesh_show_wireframe=True)

            self.vis.add_geometry(self.open3d_mesh)
            self.vis.poll_events()
            self.vis.update_renderer()
            #only process the first element
            break 