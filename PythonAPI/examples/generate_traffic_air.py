#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""

import glob
from inspect import trace
import os
import sys
from telnetlib import DO
import time
from turtle import up, update

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging
import numpy as np

from multirotor.drone import Drone # 引入飞行控制类

class Trajectory(object):
    def __init__(self):
        self.waypoints = []
        self.drones = []
        self.controllers = []
        # 飞行行驶边界
        self.x_boundary_min = -150
        self.x_boundary_max = 150
        self.y_boundary_min = -70
        self.y_boundary_max = 170
        self.z_boundary_max = 60
        self.z_boundary_min = -1

        # 出生点 # TODO： 在这里临时统一度量单位是个很大的隐患
        self.flycar_spawn_points_1 = carla.Transform(carla.Location(x=self.x_boundary_min, y=-50, z=50), carla.Rotation(pitch=0, yaw=-180*np.pi/180, roll=0))
        self.flycar_spawn_points_2 = carla.Transform(carla.Location(x=self.x_boundary_min, y=20, z=50), carla.Rotation(pitch=0, yaw=-180*np.pi/180, roll=0))
        self.flycar_spawn_points_3 = carla.Transform(carla.Location(x=self.x_boundary_min, y=140, z=50), carla.Rotation(pitch=0, yaw=-180*np.pi/180, roll=0))
        self.flycar_spawn_points_4 = carla.Transform(carla.Location(x=-41, y=self.y_boundary_max, z=50), carla.Rotation(pitch=0, yaw=-90*np.pi/180, roll=0))
        self.flycar_spawn_points_5 = carla.Transform(carla.Location(x=53, y=self.y_boundary_max, z=50), carla.Rotation(pitch=0, yaw=-90*np.pi/180, roll=0))
        self.flycar_spawn_points_6 = carla.Transform(carla.Location(x=111, y=self.y_boundary_max, z=50), carla.Rotation(pitch=0, yaw=-90*np.pi/180, roll=0))
        self.flycar_spawn_points_7 = carla.Transform(carla.Location(x=self.x_boundary_max, y=130, z=50), carla.Rotation(pitch=0, yaw=-180*np.pi/180, roll=0))
        self.flycar_spawn_points_8 = carla.Transform(carla.Location(x=self.x_boundary_max, y=5, z=50), carla.Rotation(pitch=0, yaw=-180*np.pi/180, roll=0))
        self.flycar_spawn_points_9 = carla.Transform(carla.Location(x=self.x_boundary_max, y=-55, z=50), carla.Rotation(pitch=0, yaw=-180*np.pi/180, roll=0))
        self.flycar_spawn_points_10 = carla.Transform(carla.Location(x=100, y=self.y_boundary_min, z=50), carla.Rotation(pitch=0, yaw=-90*np.pi/180, roll=0))
        self.flycar_spawn_points_11 = carla.Transform(carla.Location(x=36, y=self.y_boundary_min, z=50), carla.Rotation(pitch=0, yaw=-90*np.pi/180, roll=0))
        self.flycar_spawn_points_12 = carla.Transform(carla.Location(x=-50, y=self.y_boundary_min, z=50), carla.Rotation(pitch=0, yaw=-90*np.pi/180, roll=0))

    def _generate_start_point(self, scale):
        start_points = scale * [self.flycar_spawn_points_1, self.flycar_spawn_points_2, self.flycar_spawn_points_3, self.flycar_spawn_points_4,\
                                self.flycar_spawn_points_5, self.flycar_spawn_points_6, self.flycar_spawn_points_7, self.flycar_spawn_points_8,\
                                self.flycar_spawn_points_9, self.flycar_spawn_points_10, self.flycar_spawn_points_11, self.flycar_spawn_points_12]
        return start_points


    def generate_drone(self, num=12):
        for i in range(num):
            self.drones.append(Drone())
            self.drones[i].id = i # 按序列赋予编号
        
    def set_trajectory(self, id, trajectoryList):
        self.drones[id].trajectory = trajectoryList

    def flocking(self):
        self.drones[id].trajectory_motion()

    def destroy(self):
        if len(self.controllers) != 0:
            for i in len(self.controllers):
                self.controllers[i].destroy()

    def motion(self, id, delta_t):
        drone = self.drones[id]
        drone.controller_update(*drone.trajectory)
        l = drone.dynamics_euler_angles(delta_t)
    
    def update_state(self, id):
        drone = self.drones[id]
        ue_object_transform = self.controllers[id].get_transform()

        drone.state[0] = ue_object_transform.location.x
        drone.state[1] = ue_object_transform.location.y
        drone.state[2] = ue_object_transform.location.z
        # drone.state[3] = body_velocity.x
        # drone.state[4] = body_velocity.y
        # drone.state[5] = body_velocity.z
        drone.state[6] = -ue_object_transform.rotation.roll 
        drone.state[7] = -ue_object_transform.rotation.pitch 
        drone.state[8] = ue_object_transform.rotation.yaw *np.pi/180


    def set_state(self, id):
        drone = self.drones[id]
        ue_object = self.controllers[id]
        ue_object_transform = ue_object.get_transform()

        ue_object_transform.location.x = drone.state[0]
        ue_object_transform.location.y = drone.state[1]
        ue_object_transform.location.z = drone.state[2]
        ue_object_transform.rotation.roll = -drone.theta
        ue_object_transform.rotation.pitch = -drone.phi
        ue_object_transform.rotation.yaw = drone.psi*180/np.pi

        ue_object.set_transform(ue_object_transform)

    
    def forward_motion(self, id):
        drone = self.drones[id]
        ue_object = self.controllers[id]

        if ue_object is None: # TODO: 可以去除
            # print("NO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            return 
        # TODO: 应当以车体坐标系为基准移动而不是世界坐标系
        self.update_state(id)
        if id >= 0 and id <=2:
            drone.state[0] += 0.3
        elif id >= 3 and id <=5:
            drone.state[1] -= 0.3
        elif id >= 6 and id <=8:
            drone.state[0] -= 0.3
        elif id >= 9 and id <=11:
            drone.state[1] += 0.3

        self.set_state(id)


    def set_starting_point(self):
        pass

    def _arrive_border(self, id):
        drone = self.drones[id]
        if (drone.state[0] < self.x_boundary_min-5 or drone.state[0] > self.x_boundary_max+5 \
            or drone.state[1] < self.y_boundary_min-5 or drone.state[1] > self.y_boundary_max+5 \
            or drone.state[2] < self.z_boundary_min or drone.state[2] > self.z_boundary_max+5):
            print(id, "已到达边界")
            return True
        else:
            return False

    # def destory(self, id):
    #     if self._arrive_border(id):
    #         self.drones[id] = None

    def _apply_restart(self, id, x, y, z, yaw):
        drone = self.drones[id]
        ue_object = self.controllers[id]
        ue_object_transform = ue_object.get_transform()

        drone.state[0] = x
        drone.state[1] = y
        drone.state[2] = z
        drone.theta = 0   # TODO: 出生点默认是水平姿态
        drone.phi = 0
        drone.psi = yaw *np.pi/180

        ue_object_transform.location.x = drone.state[0]
        ue_object_transform.location.y = drone.state[1]
        ue_object_transform.location.z = drone.state[2]
        ue_object_transform.rotation.roll = -drone.theta
        ue_object_transform.rotation.pitch = -drone.phi
        ue_object_transform.rotation.yaw = drone.psi*180/np.pi

        ue_object.set_transform(ue_object_transform)


    def restart(self, id):
        if self._arrive_border(id):
            # TODO： 这里应当使用更好的重生编队方法
            # start_x = 0
            # start_y = 0
            # start_z = 50
            # start_dir = 0 


            # if id == 0:
            #     start_x = self.x_boundary_min
            #     start_y = -40
            #     start_z = 50
            #     start_dir = 0 
            # elif id == 1:
            #     start_x = self.x_boundary_min
            #     start_y = -10
            #     start_z = 50
            #     start_dir = 0 
            drone = self.drones[id]
            start_x = drone.start_point.location.x
            start_y = drone.start_point.location.y
            start_z = drone.start_point.location.z
            start_dir = drone.start_point.rotation.yaw * 180/np.pi


            self._apply_restart(id, start_x, start_y, start_z, start_dir)


def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=50,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=10,
        type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='2',
        help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--asynch',
        action='store_true',
        help='Activate asynchronous mode execution')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Activate hybrid mode for Traffic Manager')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '--seedw',
        metavar='S',
        default=0,
        type=int,
        help='Set the seed for pedestrians module')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enable automatic car light management')
    argparser.add_argument(
        '--hero',
        action='store_true',
        default=False,
        help='Set one of the vehicles as hero')
    argparser.add_argument(
        '--respawn',
        action='store_true',
        default=False,
        help='Automatically respawn dormant vehicles (only in large maps)')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        default=False,
        help='Activate no rendering mode')

    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    np.random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        if args.respawn:
            traffic_manager.set_respawn_dormant_vehicles(True)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(70.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        settings = world.get_settings()
        if not args.asynch:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            else:
                synchronous_master = False
        else:
            print("You are currently in asynchronous mode. If this is a traffic simulation, \
            you could experience some issues. If it's not working correctly, switch to synchronous \
            mode by using traffic_manager.set_synchronous_mode(True)")

        if args.no_rendering:
            settings.no_rendering_mode = True
        world.apply_settings(settings)

        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)

        if args.safe:
            blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            np.random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        hero = args.hero
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = np.random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = np.random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = np.random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if hero:
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        # Set automatic vehicle lights update if specified
        if args.car_lights_on:
            all_vehicle_actors = world.get_actors(vehicles_list)
            for actor in all_vehicle_actors:
                traffic_manager.update_vehicle_lights(actor, True)

        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        percentagePedestriansRunning = 0.0      # how many pedestrians will run
        percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
        if args.seedw:
            world.set_pedestrians_seed(args.seedw)
            np.random.seed(args.seedw)
        # 1. take all the np.random locations to spawn
        spawn_points = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = np.random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (np.random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put together the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)


        # -------------
        # Spawn FlyCars
        # -------------
        traj_manage = Trajectory()
        flycar_scale = 5 # TODO: 应当用更精细的数量控制
        traj_manage.generate_drone(num = flycar_scale*12)
        # flycar_blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        flycar_blueprints = get_actor_blueprints(world, 'uav', "all")
        # traj_manage.destroy() # TODO: 销毁所有车辆

        # TODO: 急具隐患的手动添加
        flycar_spawn_points = traj_manage._generate_start_point(flycar_scale)

        # for id, spawn_point in enumerate(spawn_points):
        for id, spawn_point in enumerate(flycar_spawn_points):
            # print(spawn_point.location)
            traj_manage.controllers.append(world.try_spawn_actor(flycar_blueprints[0], spawn_point))
            traj_manage.drones[id].start_point = spawn_point


            if traj_manage.controllers[id] != None:
                traj_manage.controllers[id].set_simulate_physics(False)
        
        # print(len(flycar_spawn_points), len(traj_manage.drones), len(traj_manage.controllers))


        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if args.asynch or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))


        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

        # Example of how to use Traffic Manager parameters
        traffic_manager.global_percentage_speed_difference(30.0)

        while True:
            for id in range(len(traj_manage.drones)):
                # print(traj_manage.drones[id].id)
                traj_manage.forward_motion(id)
                traj_manage.restart(id)


            if not args.asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

    finally:

        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
