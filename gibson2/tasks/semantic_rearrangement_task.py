from gibson2.tasks.task_base import BaseTask
import pybullet as p
from gibson2.scenes.igibson_indoor_scene import InteractiveIndoorScene
from gibson2.termination_conditions.max_collision import MaxCollision
from gibson2.termination_conditions.timeout import Timeout
from gibson2.termination_conditions.out_of_bound import OutOfBound
from gibson2.reward_functions.null_reward import NullReward
from gibson2.objects.custom_wrapped_object import CustomWrappedObject
import gibson2.external.pybullet_tools.utils as PBU

import logging
import numpy as np
from collections import OrderedDict


class SemanticRearrangementTask(BaseTask):
    """
    Semantic Rearrangement Task
    The goal is to sort or gather multiple semantically distinct objects

    Args:
        env (BaseEnv): Environment using this task
        #objects (list of CustomWrappedObject): Object(s) to use for this task
        goal_pos (3-array): (x,y,z) cartesian global coordinates for the goal location
        randomize_initial_robot_pos (bool): whether to randomize initial robot position or not. If False,
            will selected based on pos_range specified in the config
    """

    def __init__(self, env, goal_pos=(0,0,0), randomize_initial_robot_pos=True):
        super().__init__(env)
        # Currently, this should be able to be done in either a gibson or igibson env
        # assert isinstance(env.scene, InteractiveIndoorScene), \
        #     'room rearrangement can only be done in InteractiveIndoorScene'
        self.termination_conditions = [
            MaxCollision(self.config),
            Timeout(self.config),
            OutOfBound(self.config),
        ]
        # Goal
        self.goal_pos = np.array(goal_pos)
        # Reward-free task currently
        self.reward_functions = [
            NullReward(self.config),
        ]
        self.floor_num = 0
        self.sampling_args = self.config.get("sampling", {})
        # Robot
        self.robot_body_id = env.robots[0].robot_ids[0]
        self.robot_gripper_joint_ids = env.robots[0].gripper_joint_ids
        # Objects
        self.target_objects = self._create_objects(env=env)
        self.target_objects_id = {k: i for i, k in enumerate(self.target_objects.keys())}
        self.target_object = None                   # this is the current active target object for the current episode
        self.exclude_body_ids = []                  # will include all body ids belonging to any items that shouldn't be outputted in state
        # Other internal vars
        self.randomize_initial_robot_pos = randomize_initial_robot_pos
        self.init_pos_range = np.array(self.config.get("pos_range", np.zeros((2,3))))
        self.init_rot_range = np.array(self.config.get("rot_range", np.zeros(2)))
        self.target_object_init_pos = None                        # will be initial x,y,z sampled placement in active episode
        # Observation mode
        self.task_obs_format = self.config.get("task_obs_format", "global") # Options are global, egocentric
        assert self.task_obs_format in {"global", "egocentric"}, \
            f"Task obs format must be one of: [global, egocentric]. Got: {self.task_obs_format}"
        # Store all possible scene locations for the target object
        self.target_locations = None
        self.target_location = None         # Where the object is actually located this episode
        self.target_location_ids = None     # maps location names to id number

        # Store env
        self.env = env

    def _create_objects(self, env):
        """
        Helper function to create objects

        Returns:
            dict: objects mapped from name to BaseObject instances
        """
        objs = OrderedDict()
        # Loop over all objects from the config file and load them
        for obj_config in self.config.get("objects", []):
            obj = CustomWrappedObject(env=env, only_top=self.sampling_args.get("only_top", False), **obj_config)
            # Import this object into the simulator
            env.simulator.import_object(obj=obj, class_id=obj.class_id)
            # Store a reference to this object
            objs[obj_config["name"]] = obj

        # Return created objects
        return objs

    def reset_scene(self, env):
        """
        Reset all scene objects as well as objects belonging to this task.

        :param env: environment instance
        """
        # Only reset scene objects if we're in an interactive scene
        if type(env.scene).__name__ == "InteractiveIndoorScene":
            env.scene.reset_scene_objects()
            env.scene.force_wakeup_scene_objects()

        env.simulator.sync()

        # Sample new target object and reset exlude body ids
        self.target_object = np.random.choice(list(self.target_objects.values()))
        self.exclude_body_ids = []

        # Reset objects belonging to this task specifically
        for obj_name, obj in self.target_objects.items():
            # Only sample pose if this is the actual active target object
            if self.target_object.name == obj_name:
                pos, ori = obj.sample_pose()
                self.target_object_init_pos = np.array(pos)
            else:
                # Otherwise, we'll remove the object from the scene and exclude its body ids from the state
                pos, ori = [30, 30, 30], [0, 0, 0, 1]
                self.exclude_body_ids.append(self.target_object.body_id)
            obj.set_position_orientation(pos, ori)
        p.stepSimulation()

        # Store location info
        self.update_location_info()

    def update_location_info(self):
        """
        Helper function to update location info based on current target object
        """
        # Store relevant location info
        self.target_locations = {
            k: self.env.scene.objects_by_name[k] for k in self.target_object.sample_at.keys()
        }
        location_names = list(self.target_locations.keys())
        self.target_location = min(
            self.target_locations.keys(),
            key=lambda x: np.linalg.norm(self.target_object.get_position()[:2] - self.target_locations[x].init_pos[:2]))
        self.target_location_ids = {
            name: i for i, name in enumerate(location_names)
        }

    def sample_initial_pose(self, env):
        """
        Sample robot initial pose

        :param env: environment instance
        :return: initial pose
        """
        if self.randomize_initial_robot_pos:
            _, initial_pos = env.scene.get_random_point(floor=self.floor_num)
        else:
            initial_pos = np.random.uniform(self.init_pos_range[0], self.init_pos_range[1])
        initial_orn = np.array([0, 0, np.random.uniform(self.init_rot_range[0], self.init_rot_range[1])])
        return initial_pos, initial_orn

    def reset_agent(self, env):
        """
        Reset robot initial pose.
        Sample initial pose, check validity, and land it.

        :param env: environment instance
        """
        reset_success = False
        max_trials = 100

        # cache pybullet state
        # TODO: p.saveState takes a few seconds, need to speed up
        state_id = p.saveState()
        for _ in range(max_trials):
            initial_pos, initial_orn = self.sample_initial_pose(env)
            reset_success = env.test_valid_position(
                env.robots[0], initial_pos, initial_orn)
            p.restoreState(state_id)
            if reset_success:
                break

        if not reset_success:
            logging.warning("WARNING: Failed to reset robot without collision")

        env.land(env.robots[0], initial_pos, initial_orn)
        p.removeState(state_id)

        for reward_function in self.reward_functions:
            if reward_function is not None:
                reward_function.reset(self, env)

    def get_task_obs(self, env):
        """
        Get task-specific observation, including goal position, current velocities, etc.

        :param env: environment instance
        :return: task-specific observation
        """
        # Construct task obs -- consists of 3D locations of active target object
        task_obs = OrderedDict()
        obs_cat = []
        obj_dist = np.array(self.target_object.get_position())
        if self.task_obs_format == "egocentric":
            obj_dist -= np.array(env.robots[0].get_eef_position())
        task_obs[self.target_object.name] = obj_dist
        obs_cat.append(obj_dist)
        # TODO: Make this not hardcoded (pull furniture names from cfg instead)
        # table_body_id = env.scene.objects_by_name["desk_76"].body_id[0]
        # table_drawer1_joint = PBU.get_joint(body=table_body_id, joint_or_name="desk_76_joint_0")
        # table_drawer1_joint_pos = PBU.get_joint_position(body=table_body_id, joint=table_drawer1_joint)
        # task_obs["furniture_joints"] = np.array([table_drawer1_joint_pos])
        # obs_cat.append(task_obs["furniture_joints"])
        # Add concatenated obs also
        task_obs["object-state"] = np.concatenate(obs_cat)
        # Add task id
        task_id_one_hot = np.zeros(len(self.target_objects.keys()))
        task_id_one_hot[self.target_objects_id[self.target_object.name]] = 1
        task_obs["task_id"] = task_id_one_hot
        # Add location -- this is ID of object robot is at if untucked, else returns -1 (assumes we're not at a location)
        if env.robots[0].tucked:
            task_obs["robot_location"] = -1
        else:
            robot_pos = np.array(env.robots[0].get_eef_position())
            robot_location = min(
                self.target_locations.keys(),
                key=lambda x: np.linalg.norm(robot_pos[:2] - self.target_locations[x].init_pos[:2]))
            task_obs["robot_location"] = self.target_location_ids[robot_location]
        # Object location
        task_obs["target_obj_location"] = self.target_location_ids[self.target_location]

        return task_obs

    def set_conditions(self, conditions):
        """
        Method to override task conditions (e.g.: target object), useful in cases such as playing back
            from demonstrations

        Args:
            conditions (dict): Keyword-mapped arguments to set internally
        """
        # Set target object
        self.set_target_object(identifier=conditions["task_id"])

    def set_target_object(self, identifier):
        """
        Manually sets the target object for the current episode. Useful for, e.g., if deterministically resetting the
        state.

        Args:
            identifier (str or int): Either the ID (as mapped by @self.target_objects_id) or object name to set
                as the target
        """
        if type(identifier) is int:
            obj_name = list(self.target_objects.keys())[identifier]
        elif type(identifier) is str:
            obj_name = identifier
        else:
            raise TypeError("Identifier must be either an int or str!")

        self.target_object = self.target_objects[obj_name]

        # Update location info as well
        self.update_location_info()

    def update_target_object_init_pos(self):
        """
        Function to manually update the initial target object position. Useful for, e.g., if deterministically playing
        back from hard-coded states
        """
        self.target_object_init_pos = self.target_object.get_position()

    def check_success(self):
        """
        Checks various success states and returns the keyword-mapped values

        Returns:
            dict: Success criteria mapped to bools
        """
        # Task is considered success if target object is touching both gripper fingers and lifted by small margin
        collisions = list(p.getContactPoints(bodyA=self.target_object.body_id, bodyB=self.robot_body_id))
        touching_left_finger, touching_right_finger = False, False
        task_success = False
        if self.target_object.get_position()[2] - self.target_object_init_pos[2] > 0.05:
            # Object is lifted, now check for gripping contact
            for item in collisions:
                if touching_left_finger and touching_right_finger:
                    # No need to continue iterating
                    task_success = True
                    break
                # check linkB to see if it matches either gripper finger
                if item[4] == self.robot_gripper_joint_ids[0]:
                    touching_right_finger = True
                elif item[4] == self.robot_gripper_joint_ids[1]:
                    touching_left_finger = True

        # Compose and success dict
        success_dict = {"task": task_success}

        # Return dict
        return success_dict

    def sync_state(self):
        """
        Helper function to synchronize internal state variables with actual sim state. This might be necessary
        where state mismatches may occur, e.g., after a direct sim state setting where env.step() isn't explicitly
        called.
        """
        # We need to update target object if there isn't an internal one
        # or if we detect that it's currently out of the scene
        if self.target_object is None or np.linalg.norm(self.target_object.get_position()) > 45:
            # Iterate over the target objects; we know the current active object is the one that's not out of the scene
            for obj in self.target_objects.values():
                if np.linalg.norm(obj.get_position()) < 45:
                    # This is the target object, update it and break
                    self.target_object = obj
                    # Also update location info
                    self.update_location_info()
                    break
