#!/usr/bin/env python

import os

import pandas
import rospy
import rospkg
from ruamel.yaml import YAML
from argparse import ArgumentParser
import cv2
from sensor_msgs.msg import Image
import rosbag

from cv_bridge import CvBridge, CvBridgeError
import tf2_py
from tf.transformations import euler_from_quaternion, quaternion_matrix, euler_from_matrix, quaternion_from_matrix

from numpy import savetxt
import numpy as np
import matplotlib.pyplot as plt  # Just for debug.

import msgpack
import msgpack_numpy as m

import math
import time

import collections

m.patch()

import pandas as pd
import subprocess
import yaml


#https://stackoverflow.com/questions/41493282/in-python-pandas-how-can-i-re-sample-and-interpolate-a-dataframe
#https://stackoverflow.com/questions/48068938/set-new-index-for-pandas-dataframe-interpolating
def reindex_and_interpolate(df, new_index):
    return df.reindex(df.index.union(new_index)).interpolate(method='index', limit_direction='both').loc[new_index]

def reindex_union(df, new_index, method='nearest'):
    # return df.reindex(df.index.union(new_index), method=method)
    return df.reindex(new_index, method=method, tolerance=0.5)

#From rosbag_pandas
def get_bag_info(bag_file):
    '''Get uamle dict of the bag information
    by calling the subprocess -- used to create correct sized
    arrays'''
    # Get the info on the bag
    bag_info = yaml.load(subprocess.Popen(
        ['rosbag', 'info', '--yaml', bag_file],
        stdout=subprocess.PIPE).communicate()[0])
    return bag_info


def get_length(topics, yaml_info):
    '''
    Find the length (# of rows) in the created dataframe
    '''
    length_per_topic = {}
    total = 0
    info = yaml_info['topics']
    for topic in topics:
        for t in info:
            if t['topic'] == topic:
                total = total + t['messages']
                length_per_topic[topic] = t['messages']
                break
    return total, length_per_topic


def msg_to_joint_mechanical_power(state_msg):
    pw = np.array(state_msg.joints.effort) * np.array(state_msg.joints.velocity)
    pw = np.clip(pw, 0.0, None)
    pw = np.sum(pw)
    return pw


def msg_to_joint_positions(state_msg):
    return state_msg.joints.position


def msg_to_joint_velocities(state_msg):
    return state_msg.joints.velocity


def msg_to_body_lin_vel(state_msg):
    return np.array([state_msg.twist.twist.linear.x, state_msg.twist.twist.linear.y, state_msg.twist.twist.linear.z])


def msg_to_body_ang_vel(state_msg):
    return np.array([state_msg.twist.twist.angular.x, state_msg.twist.twist.angular.y, state_msg.twist.twist.angular.z])


def msg_to_grav_vec(state_msg):
    quat = (state_msg.pose.pose.orientation.x, state_msg.pose.pose.orientation.y, state_msg.pose.pose.orientation.z,
            state_msg.pose.pose.orientation.w)
    mat = quaternion_matrix(quat)
    return mat[2, :3]


def msg_to_command(command_msg):
    # # lin_dir = np.array([command_msg.twist.linear.x, command_msg.twist.linear.y])
    # # norm = np.linalg.norm(lin_dir)
    # # if norm > 0.01:
    # #     lin_dir = lin_dir / norm
    pose = [command_msg.twist.linear.x, command_msg.twist.linear.y, command_msg.twist.linear.z, command_msg.twist.angular.z]
    # pose = [command_msg.twist.linear.x, command_msg.twist.linear.y, command_msg.twist.angular.z]
    return pose




def msg_to_pose(tf_msg):
    quat = (
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w)
    euler = euler_from_quaternion(quat)
    pose = [tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z,
            quat[0],
            quat[1],
            quat[2],
            quat[3]]
    # mat = quaternion_matrix(quat)
    return pose


def msg_to_rotmat(tf_msg):
    quat = (
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w)
    mat = quaternion_matrix(quat)
    return mat[:3, :3]



class Data:
    def __init__(self):
        self.images = []
        self.poses = []

class DataBuffer:
    def __init__(self):
        self.indices = {}
        self.data = {}
        self.data_id = {}

    def append(self, stamp, data, name):

        if name in self.data:
            self.data[name].append(data)
            self.indices[name].append(stamp)
            self.data_id[name].append(self.data_id[name][-1] + 1)
        else:
            self.data[name] = [data]
            self.indices[name] = [stamp]
            self.data_id[name] = [0]


def extractData(file_name, out_dir):
    # Make sure output directory exists.
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)


    # Load parameters
    # ELEVATION_MAP_TOPIC = cfg['ELEVATION_MAP_TOPIC']

    STATE_TOPIC = '/state_estimator/anymal_state'
    COMMAND_TOPIC = cfg['COMMAND_TOPIC']

    TF_BASE = cfg['TF_BASE']
    TF_POSE = cfg['TF_POSE']

    state_topics = [STATE_TOPIC, COMMAND_TOPIC]
    img_topics = [ELEVATION_MAP_TOPIC]
    img_topics += ['/' + ELEVATION_MAP_TOPIC]  #weird corner case
    img_topics += [CAM_PREFIX + suffix for suffix in CAM_SUFFIXES]
    img_topics += ['/' + CAM_PREFIX + suffix for suffix in CAM_SUFFIXES]

    dt = rospy.Duration.from_sec(cfg['dt'])
    resample_freq = str(dt.to_sec()) + ' S'
    local_map_shape = cfg['local_map_shape']

    # dump YAML CFG
    cfg['file_name'] = file_name
    with open(out_dir + "/data_extraction.yaml", 'w') as yaml_file:
        YAML().dump(cfg, yaml_file)

    yaml_info = get_bag_info(file_name)


    print('Parsing bag \'' + file_name + '\'...')
    bag = rosbag.Bag(file_name)

    command_stamps = []
    for topic, msg, t in bag.read_messages(COMMAND_TOPIC):
        command = msg_to_command(msg)  # in baseframe
        if np.sum(np.abs(command)) > 0:
            command_stamps.append(msg.header.stamp)

    command_stamps = list(set(command_stamps))  # remove duplicates
    command_stamps.sort()

    # Divide command_stamps into sub sequences
    time_stamps_sequences = []
    continuous_stamps = []
    min_traj_len = cfg['min_traj_len'] * dt.to_sec()
    max_traj_len = cfg['max_traj_len'] * dt.to_sec()

    for i in range(1, len(command_stamps)):
        dt_ = (command_stamps[i] - command_stamps[i - 1]).to_sec()

        seq_length = 0
        if len(continuous_stamps) > 1:
            seq_length = (continuous_stamps[-1] - continuous_stamps[0]).to_sec()

        if  dt_ > cfg['dt'] or seq_length >= max_traj_len or i == (len(command_stamps) - 1):
            if seq_length >= min_traj_len:
                time_stamps_sequences.append(continuous_stamps)
            continuous_stamps = []
        continuous_stamps.append(command_stamps[i])

    print("Number of sub sequences: {}".format(len(time_stamps_sequences)))
    # for seq in time_stamps_sequences:
    #     print(len(seq), "test")

    tf_buffer = tf2_py.BufferCore(rospy.Duration.from_sec(max_traj_len * dt.to_sec() * 1.1))

    # first tf.
    for topic, msg, t in bag.read_messages(topics=['/tf'], end_time=time_stamps_sequences[0][0]):
        if topic == '/tf':
            for transform in msg.transforms:
                if transform.header.frame_id in TF_POSE:
                    tf_buffer.set_transform(transform, 'rosbag')


    # Process trajectories
    file_idx = 0

    for traj in time_stamps_sequences:
        start_time = traj[0]
        end_time = traj[-1]

        # first fill up tf buffer.
        for topic, msg, t in bag.read_messages(topics=['/tf'], start_time=start_time,
                                               end_time=end_time):
            for transform in msg.transforms:
                if transform.header.frame_id in TF_POSE:
                    tf_buffer.set_transform(transform, 'rosbag')

        # Update states
        command_data = DataBuffer()
        pose_data = DataBuffer()
        velocity_data = DataBuffer()
        state_data = DataBuffer()
        image_data = DataBuffer()
        for topic, msg, t in bag.read_messages(topics=img_topics + state_topics, start_time=start_time,
                                               end_time=end_time):

            # command
            if topic == COMMAND_TOPIC:
                command = msg_to_command(msg)  # in baseframe
                command_data.append(msg.header.stamp.to_sec(), command, 'base')

            # ANYmal state
            if topic == STATE_TOPIC:
                lin_vel = msg_to_body_lin_vel(msg)  # in baseframe
                ang_vel = msg_to_body_ang_vel(msg)
                joint_pos = msg_to_joint_positions(msg)

                vel_concat = np.concatenate([lin_vel, ang_vel])

                velocity_data.append(msg.header.stamp.to_sec(), vel_concat, 'base')
                state_data.append(msg.header.stamp.to_sec(), joint_pos, 'joint_position')

            # RGB images.
            if CAM_PREFIX in topic:
                success, img_id = getImageId(topic, CAM_SUFFIXES)
                img = rgb_msg_to_image(msg, cfg['max_img_width'], img_id in CAM_RBSWAP)

                if success:
                    image_data.append(msg.header.stamp.to_sec(), img,  'rgb_' + img_id)
                else:
                    print('Unknown RGB image topic: ' + topic)

            # Elevation map
            if ELEVATION_MAP_TOPIC in topic:
                grid_map = GridMapFromMessage(msg)
                image_data.append(msg.info.header.stamp.to_sec(), grid_map, 'elevation_map')

        ## Update pandas dfs
        # Update poses & commands
        for i, stamp in enumerate(command_data.indices['base']):
            stamp_ros = rospy.Time.from_sec(stamp)
            command_base = command_data.data['base'][i]

            for key in TF_POSE:
                if (tf_buffer.can_transform_core(key, TF_BASE, stamp_ros)[0]):
                    tf = tf_buffer.lookup_transform_core(key, TF_BASE, stamp_ros)
                    pose = msg_to_pose(tf)  # pose in fixed ref frame (odom or map)
                    pose_data.append(stamp, pose, key)

                    R_T = np.zeros([2, 3, 3])
                    R_T[:] = msg_to_rotmat(tf)

                    command_transformed = command_base.copy()
                    command_transformed[:3] = R_T[0] @ command_base[:3]
                    command_data.append(stamp, command_transformed, key)

        # Transform velocities
        for i, stamp in enumerate(velocity_data.indices['base']):
            stamp_ros = rospy.Time.from_sec(stamp)
            velocity_base = velocity_data.data['base'][i]

            for key in TF_POSE:
                if tf_buffer.can_transform_core(key, TF_BASE, stamp_ros)[0]:
                    tf = tf_buffer.lookup_transform_core(key, TF_BASE, stamp_ros)

                    R_T = np.zeros([2, 3, 3])
                    R_T[:] = msg_to_rotmat(tf)
                    vel_transformed = np.matmul(R_T, np.reshape(velocity_base, [2, 3, 1]))
                    vel_transformed = vel_transformed.reshape([6])
                    velocity_data.append(stamp, vel_transformed,  key)


        # PROCESS AND SAVE
        # print(command_data.data.keys())
        # print(pose_data.data.keys())
        # print(state_data.data.keys())
        # print(velocity_data.data.keys())
        # print(image_data.indices.keys())

        cmd_dfs = []
        for key in command_data.data_id.keys():
            df = pd.DataFrame(data=command_data.data[key], index=command_data.indices[key])
            df.index = pd.to_datetime(df.index, unit='s')
            cmd_dfs.append(df)
        cmd_df = pd.concat(cmd_dfs, axis=1, keys=command_data.data_id.keys())
        cmd_df = cmd_df.resample(resample_freq).apply(lambda x: x.mean() if x.isnull().sum() == 0 else np.nan)
        resampled_idx = cmd_df.index
        # resampled_idx = resampled_idx.drop_duplicates(keep='first')

        img_dfs = []
        for key in image_data.data_id.keys():
            df = pd.DataFrame(data=image_data.data_id[key], index=image_data.indices[key])
            df.index = pd.to_datetime(df.index, unit='s')
            df = df[~df.index.duplicated(keep='first')]

            df = df.reindex(resampled_idx, method='ffill', tolerance=str(dt * 0.5) + 's')  # ffill
            img_dfs.append(df)
        img_df = pd.concat(img_dfs, axis=1, keys=image_data.data_id.keys())

        pose_dfs = []
        for key in pose_data.data_id.keys():
            df = pd.DataFrame(data=pose_data.data[key], index=pose_data.indices[key])
            df.index = pd.to_datetime(df.index, unit='s')
            df = df[~df.index.duplicated(keep='first')]
            df = df.reindex(resampled_idx, method='ffill', tolerance=str(dt * 0.5) + 's')  # ffill
            pose_dfs.append(df)

        pose_df = pd.concat(pose_dfs, axis=1, keys=pose_data.data_id.keys())


        state_dfs = []
        for key in state_data.data_id.keys():
            df = pd.DataFrame(data=state_data.data[key], index=state_data.indices[key])
            df.index = pd.to_datetime(df.index, unit='s')
            df = df[~df.index.duplicated(keep='first')]
            df = df.reindex(resampled_idx, method='ffill', tolerance=str(dt * 0.5) + 's')  # ffill
            state_dfs.append(df)
        state_df = pd.concat(state_dfs, axis=1, keys=state_data.data_id.keys())

        vel_dfs = []
        for key in velocity_data.data_id.keys():
            df = pd.DataFrame(data=velocity_data.data[key], index=velocity_data.indices[key])
            df.index = pd.to_datetime(df.index, unit='s')
            df = df[~df.index.duplicated(keep='first')]

            # df.loc[resampled_idx[0]] = np.nan
            # df = df.resample(resample_freq).mean()
            # df = df.reindex(resampled_idx, method='nearest')
            df = df.reindex(resampled_idx, method='ffill', tolerance=str(dt * 0.5) + 's')  # ffill
            vel_dfs.append(df)

        vel_df = pd.concat(vel_dfs, axis=1, keys=velocity_data.data_id.keys())

        transition_vel_dfs = []
        for key in velocity_data.data_id.keys():
            df = pd.DataFrame(data=velocity_data.data[key], index=velocity_data.indices[key])
            df.index = pd.to_datetime(df.index, unit='s')
            df = df[~df.index.duplicated(keep='first')]

            df.loc[resampled_idx[0]] = np.nan
            df = df.resample(resample_freq).mean()
            df = df.reindex(resampled_idx, method='nearest')
            transition_vel_dfs.append(df)

        transition_vel_df = pd.concat(transition_vel_dfs, axis=1, keys=velocity_data.data_id.keys())
        # transition_vel_df.to_csv(out_dir+'/transition_vel_df.csv')


        total_df = pd.concat([cmd_df, img_df, pose_df, vel_df, state_df, transition_vel_df],
                             axis=1,
                             keys=['cmd', 'img', 'pose', 'velocity', 'state', 'transition_velocity'])
        total_df.dropna(axis=0, inplace=True, how='any')

        previous_index = total_df.index[0]
        discontinuity = [0]
        list_of_dfs = []

        for i, idx in enumerate(total_df.index):
            diff = (idx - previous_index).total_seconds()
            if diff > dt.to_sec():
                discontinuity.append(i)
            previous_index = idx

        if len(discontinuity) > 0:
            for i in range(len(discontinuity)):
                if i == len(discontinuity) - 1:
                    df = total_df.iloc[discontinuity[i]:]
                else:
                    df = total_df.iloc[discontinuity[i]:discontinuity[i + 1]]
                print(np.array(df).shape)
                list_of_dfs.append(df)

        #SAVE
        for df in list_of_dfs:
            out_dat = dict()
            out_dat['commands'] = {}
            out_dat['states'] = {'poses': {}, 'velocities': {}}
            out_dat['transitions'] = {'velocities': {}}
            out_dat['images'] = {}

            cmd_keys = df['cmd'].columns.remove_unused_levels().levels[0]
            pose_keys = df['pose'].columns.remove_unused_levels().levels[0]
            velocity_keys = df['velocity'].columns.remove_unused_levels().levels[0]
            img_keys = df['img'].columns.remove_unused_levels().levels[0]

            print("img_keys", img_keys)


            transition_vel_keys = df['transition_velocity'].columns.remove_unused_levels().levels[0]

            for key in cmd_keys:
                out_dat['commands'][key] = np.array(df['cmd'][key])

            for key in pose_keys:
                out_dat['states']['poses'][key] = np.array(df['pose'][key])

            for key in velocity_keys:
                out_dat['states']['velocities'][key] = np.array(df['velocity'][key])

            for key in transition_vel_keys:
                out_dat['transitions']['velocities'][key] = np.array(df['transition_velocity'][key])

            joint_position = np.array(df['state']['joint_position'])

            for key in img_keys:
                out_dat['images'][key] = []

                if 'rgb' in key:
                    for image_idx in np.array(df['img'][key]):
                        img = np.moveaxis(image_data.data[key][int(image_idx)], 2, 0)
                        out_dat['images'][key].append(img)

                if 'elevation' in key:
                    out_dat['collision'] = []
                    seq_length = out_dat['states']['poses']['odom'].shape[0]

                    for i, image_idx in enumerate(np.array(df['img'][key])):
                        image_idx = int(image_idx)

                        elev_map = image_data.data[key][image_idx]

                        pose = out_dat['states']['poses']['odom'][i]
                        yaw = euler_from_quaternion(pose[3:])[2]
                        local_map = elev_map.getLocalMap(pose[:3], yaw, local_map_shape)
                        out_dat['images'][key].append(local_map)

                        # Collision detection using Raisim
                        map_raisim = elev_map.map_matrix.transpose()
                        map_raisim = np.flip(map_raisim)
                        if raisim_objects['terrain'] is not None:
                            raisim_objects['world'].removeObject(raisim_objects['terrain'])

                        raisim_objects['terrain'] = raisim_objects['world'].addHeightMap(
                            x_scale=elev_map.size[0] * elev_map.resolution,
                            y_scale=elev_map.size[1] * elev_map.resolution,
                            x_samples=elev_map.size[0],
                            y_samples=elev_map.size[1],
                            x_center=elev_map.position[0],
                            y_center=elev_map.position[1],
                            heights=map_raisim.flatten()
                        )

                        q = np.zeros(19)
                        q[:3] = pose[:3]
                        q[3] = pose[6]
                        q[4:7] = pose[3:6]
                        q[7:] = joint_position[i]

                        raisim_objects['anymal'].setGeneralizedCoordinate(q)
                        raisim_objects['world'].integrate1()
                        num_non_foot_contacts = 0

                        for contact in raisim_objects['anymal'].getContacts():
                            if contact.getlocalBodyIndex() not in [3, 6, 9, 12]:
                                num_non_foot_contacts += 1

                        out_dat['collision'].append(num_non_foot_contacts)
                        # map_local = cv2.resize(local_map[0], (500, 500), interpolation=cv2.INTER_NEAREST)
                        # cv2.imshow('map_local', (map_local - np.min(map_local))/(np.max(map_local) - np.min(map_local)))
                        # cv2.waitKey()

                    out_dat['collision'] = np.array(out_dat['collision'])
                out_dat['images'][key] = np.array(out_dat['images'][key])

            out_file = os.path.join(out_dir, str(file_idx))
            out_file += '.msgpack'
            file_idx += 1
            with open(out_file, "wb") as outfile:
                file_dat = msgpack.packb(out_dat)
                outfile.write(file_dat)

    print('Extracting states complete.')


def main():
    parser = ArgumentParser()
    parser.add_argument('--bagfile', required=True, help='Bag file or directory with bag files to extract.')
    parser.add_argument('--outdir', required=True, help='Output directory.')

    args = parser.parse_args()
    isdir = os.path.isdir(args.bagfile)

    # launch raisim server (for visualization)
    # server = raisim.RaisimServer(raisim_objects['world'])
    # server.launchServer(8080)

    # Get all bag files
    bagfiles = []
    if isdir:
        for root, dirs, files in os.walk(args.bagfile):
            for file in files:
                if file.endswith('.bag'):
                    bagfiles.append(os.path.join(root, file))
    else:
        assert args.bagfile.endswith('.bag'), 'Specified file is not a bag file.'
        bagfiles.append(args.bagfile)

    for i, bagfile in enumerate(bagfiles):
        print('Extracting file ' + str(i + 1) + '/' + str(len(bagfiles)))
        # Get file basename for subfolder.
        basename = os.path.splitext(os.path.basename(bagfile))[0]
        # print(basename)
        extractData(bagfile, os.path.join(args.outdir, basename))

    print("DONE!")

if __name__ == '__main__':
    main()
