import os

import pybullet as p

import igibson
from igibson.objects.stateful_object import StatefulObject
import numpy as np


class Pedestrian(StatefulObject):
    """
    Pedestiran object
    """

    def __init__(self, style="standing", pos=[0, 0, 0], scale=1.0, visual_only=False, **kwargs):
        super(Pedestrian, self).__init__(**kwargs)
        self.collision_filename = os.path.join(
            igibson.assets_path, "models", "person_meshes", "person_{}".format(style), "meshes", "person_vhacd.obj"
        )
        self.visual_filename = os.path.join(
            igibson.assets_path, "models", "person_meshes", "person_{}".format(style), "meshes", "person.obj"
        )

        self.visual_only = visual_only
        self.scale = scale
        self.default_orn_euler = np.array([np.pi / 2.0, 0.0, np.pi / 2.0])
        self.cid = None
        self.pos = pos

    def _load(self, simulator):   #TODO:check whether this needs to be brought back =   , simulator):
        """
        Load the object into pybullet
        """
        collision_id = p.createCollisionShape(p.GEOM_MESH, fileName=self.collision_filename, meshScale=[self.scale] * 3)
        visual_id = p.createVisualShape(p.GEOM_MESH, fileName=self.visual_filename, meshScale=[self.scale] * 3)
        #body_id = p.createMultiBody(
        #    basePosition=[0, 0, 0], baseMass=60, baseCollisionShapeIndex=collision_id, baseVisualShapeIndex=visual_id
        #)

        if self.visual_only:
            self.body_id = p.createMultiBody(baseCollisionShapeIndex=-1,
                                        baseVisualShapeIndex=visual_id)
        else:
            self.body_id = p.createMultiBody(baseMass=60,
                                        baseCollisionShapeIndex=collision_id,
                                        baseVisualShapeIndex=visual_id)
        print("body ids", self.body_id)
        p.resetBasePositionAndOrientation(self.body_id, self.pos, p.getQuaternionFromEuler(self.default_orn_euler))  #[-0.5, -0.5, -0.5, 0.5])
#        self.cid = p.createConstraint(
#            body_id,
#            -1,
#            -1,
#            -1,
#            p.JOINT_FIXED,
#            [0, 0, 0],
#            [0, 0, 0],
#            self.pos,
#            parentFrameOrientation=[-0.5, -0.5, -0.5, 0.5],
#        )  # facing x axis

#        simulator.load_object_in_renderer(self, body_id, self.class_id, **self._rendering_params)    #TODO: check whether this line needs to be brought back

        return [self.body_id]

    def set_yaw(self, yaw):
        euler_angle = [self.default_orn_euler[0],
                       self.default_orn_euler[1],
                       self.default_orn_euler[2] + yaw]
        pos, _ = p.getBasePositionAndOrientation(self.body_id)
        p.resetBasePositionAndOrientation(
            self.body_id, pos, p.getQuaternionFromEuler(euler_angle)
        )

    def get_yaw(self):
        quat_orientation = super().get_orientation()

        # Euler angles in radians ( roll, pitch, yaw )
        euler_orientation = p.getEulerFromQuaternion(quat_orientation)

        yaw = euler_orientation[2] - self.default_orn_euler[2]
        return yaw

    def reset_position_orientation(self, pos, orn):
        """
        Reset pedestrian position and orientation by changing constraint
        """
        p.changeConstraint(self.cid, pos, orn)

    # def set_velocity(self, linear_velocity, angular_velocity):
        #global human_body_id 
        #p.resetBaseVelocity(human_body_id , linear_velocity, angular_velocity)

    def get_base_pos_and_orientation(self, ped_id):
        id = 3+ped_id
        pos, orn = p.getBasePositionAndOrientation(id)
        
        # print("id", id)
        # print("pos", pos)
        # print("orn", orn)
        return pos, orn

    def get_velocity(self, ped_id):
        """Get object bodies' velocity in the format of List[Tuple[Array[vx, vy, vz], Array[wx, wy, wz]]]"""
        #velocities = []
        #global human_body_id 

        #print("print human_body_id", human_body_id)
        id = 3 + ped_id
        #for body_id in self.get_body_ids():
        lin, ang = p.getBaseVelocity(id) #(body_id)
        #velocities.append((np.array(lin), np.array(ang)))

        return lin, ang