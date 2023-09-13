import csv
import rosbag
import pandas as pd
import gpxpy
import gpxpy.gpx
from datetime import datetime
import tf2_py
from tf.transformations import euler_from_quaternion, quaternion_matrix, euler_from_matrix, quaternion_from_matrix
import rospy
import numpy as np
import os


def msg_to_joint_positions(state_msg):
    return list(state_msg.joints.position)

def msg_to_joint_velocities(state_msg):
    return list(state_msg.joints.velocity)

def msg_to_joint_torques(state_msg):
    return list(state_msg.joints.effort)

def msg_to_body_lin_vel(state_msg):
    return np.array([state_msg.twist.twist.linear.x, state_msg.twist.twist.linear.y, state_msg.twist.twist.linear.z])


def msg_to_body_ang_vel(state_msg):
    return np.array([state_msg.twist.twist.angular.x, state_msg.twist.twist.angular.y, state_msg.twist.twist.angular.z])


def msg_to_pose(tf_msg):
    quat = (
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w)
    pose = [tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z,
            quat[0],
            quat[1],
            quat[2],
            quat[3]]
    # mat = quaternion_matrix(quat)
    return pose

def msg_to_command(command_msg):
    data_dict = {
        'command_linear_x': command_msg.twist.linear.x,
        'command_linear_y': command_msg.twist.linear.y,
        'command_angular_z': command_msg.twist.angular.z
    }
    return data_dict


class DataBuffer:
    def __init__(self, name):
        self.indices = []
        self.data = []
        self.name = name

    def append(self, stamp, data_dict):
        self.indices.append(stamp)
        self.data.append(data_dict)



def extract_data_from_bag(bag_path_info):
    # The bag file should be in the same directory as your terminal

    bag_path = bag_path_info[0]

    save_path = os.getcwd()
    save_path += "/data/" + bag_path_info[1] + "/"
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    se_topic = '/state_estimator/anymal_wheels_state'
    # actuator_topic = '/anymal_wheels_lowlevel_controller/actuator_readings'
    gps_topic = '/gps/gps'
    perf_topic = '/performance_logger/performance_state'
    command_topic = '/locomotion/vel_command'

    plan_topic = '/art_planner/path'

    dt = rospy.Duration.from_sec(0.0025)
    dt = rospy.Duration.from_sec(0.05)
    resample_freq = str(dt.to_sec()) + ' S'

    map_frame = 'odom'
    bag = rosbag.Bag(bag_path)
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    duration = end_time - start_time
    tf_buffer = tf2_py.BufferCore(rospy.Duration.from_sec(duration))

    # fill tf buffer
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        for transform in msg.transforms:
            tf_buffer.set_transform(transform, 'rosbag')


    bag = rosbag.Bag(bag_path)
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    duration = end_time - start_time

    # Extract data from rosbag
    linear_velocities_data = []
    base_positions_data = []
    command_data = DataBuffer('command')
    motion_data = DataBuffer('motion')
    joint_data = DataBuffer('joint')

    csv_path = save_path + "/se_data.csv"
    csv_path_planner = save_path + "/planner_stats.csv"
    csv_dt_path = save_path + "/planner_dts.csv"

    with open(csv_path_planner, mode='w') as csv_file:
        fieldnames = ['time']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        for topic, msg, t in bag.read_messages(topics=[se_topic, command_topic, plan_topic]):
            if topic == plan_topic:

                csv_dict = {
                    'time': t.to_sec()
                }
                # csv_dict.update(joint_state)
                writer.writerow(csv_dict)

    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(csv_path_planner)

    # Calculate the differences between consecutive values
    df['dts'] = df['time'].diff()


    df.to_csv(csv_dt_path, index=False)

    with open(csv_path, mode='w') as csv_file:
        fieldnames = ['time', 'linear_x', 'linear_y', 'linear_z', 'joint_position', 'joint_velocity', 'joint_torque']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        for topic, msg, t in bag.read_messages(topics=[se_topic, command_topic]):

            if topic == se_topic:
                lin_vel = msg_to_body_lin_vel(msg)  # in baseframe

                if tf_buffer.can_transform_core(map_frame, 'base', rospy.Time.from_sec(t.to_sec()))[0]:
                    base_pos = tf_buffer.lookup_transform_core(map_frame, 'base', rospy.Time.from_sec(t.to_sec()))

                    motion = {
                        'linear_x': lin_vel[0],
                        'linear_y': lin_vel[1],
                        'linear_z': lin_vel[2],
                        'base_x': base_pos.transform.translation.x,
                        'base_y': base_pos.transform.translation.y,
                        'base_z': base_pos.transform.translation.z
                    }

                    motion_data.append(msg.header.stamp.to_sec(), motion)

                joint_position = msg_to_joint_positions(msg)
                joint_velocity = msg_to_joint_velocities(msg)
                joint_torque = msg_to_joint_torques(msg)

                joint_state = {}
                for joint_id, position in enumerate(joint_position):
                    joint_state["joint_position" + str(joint_id)] = position

                for joint_id, velocity in enumerate(joint_velocity):
                    joint_state["joint_velocity" + str(joint_id)] = velocity

                for joint_id, torque in enumerate(joint_torque):
                    joint_state["joint_torque" + str(joint_id)] = torque

                joint_data.append(msg.header.stamp.to_sec(), joint_state)

                csv_dict = {
                    'time': t.to_sec(),
                    'linear_x': lin_vel[0],
                    'linear_y': lin_vel[1],
                    'linear_z': lin_vel[2],
                    'joint_position': joint_position,
                    'joint_velocity': joint_velocity,
                    'joint_torque': joint_torque,
                }
                # csv_dict.update(joint_state)
                writer.writerow(csv_dict)

            elif topic == command_topic:
                command = msg_to_command(msg)
                command_data.append(msg.header.stamp.to_sec(), command)

    # convert data to pandas dataframe
    def generate_df_from_buffer(data_buffer):
        df = pd.DataFrame(data=data_buffer.data, index=data_buffer.indices)
        df.index = pd.to_datetime(df.index, unit='s')
        return df

    command_df = generate_df_from_buffer(command_data)

    motion_df = generate_df_from_buffer(motion_data)
    joint_df = generate_df_from_buffer(joint_data)



    def resample_df_with_index(df, resampled_idx):
        df.index = pd.to_datetime(df.index, unit='s')
        df = df.reindex(resampled_idx, method='nearest')
        return df

    def resample_df_with_index_mean(df, resampled_idx):
        df.index = pd.to_datetime(df.index, unit='s')
        df = df.resample(resample_freq).mean()
        df = df.reindex(resampled_idx, method='nearest')
        return df

    command_df = command_df[~command_df.index.duplicated(keep='first')]
    command_df = command_df.resample(resample_freq).nearest().dropna(axis=0, how='any')
    # joint_df = joint_df.resample(resample_freq).ffill().dropna(axis=0, how='any')

    joint_df = resample_df_with_index(joint_df, command_df.index)
    motion_df = resample_df_with_index_mean(motion_df, command_df.index)

    df_concat = pd.concat([command_df, motion_df, joint_df], axis=1)
    df_concat = df_concat.dropna(axis=0, how='any')

    # Also add time column.. in a weird way
    df_concat['time'] = df_concat.index.microsecond / 1e6 +\
                        df_concat.index.second + \
                        df_concat.index.minute * 60 + \
                        df_concat.index.hour * 3600
    df_concat['time'] = df_concat['time'] - df_concat['time'][0]

    # motion_csv_path = os.path.join(save_path, 'motion_400hz2.csv')
    motion_csv_path = os.path.join(save_path, 'motion_20hz2.csv')
    df_concat.to_csv(motion_csv_path, index=False)


if __name__ == '__main__':

    # The bag file should be in the same directory as your terminal
    bag_paths = [
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/art_planner_7m_center9_0.bag','art_narrow9'),
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/art_planner_7m_center8_0.bag', 'art_narrow8'),
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/art_planner_7m_center6_0.bag', 'art_narrow6'),
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/art_planner_7m_center5_0.bag', 'art_narrow5'),
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/art_planner_10m_center8_0.bag', 'art_wide8'),
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/art_planner_10m_center7_0.bag', 'art_wide7'),
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/art_planner_10m_center6_0.bag', 'art_wide6'),
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/art_planner_10m_center5_0.bag', 'art_wide5'),
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/ours_1_0.bag', 'ours1'),
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/ours_5_0.bag', 'ours5'),
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/ours_6_0.bag', 'ours6'),
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/ours_8_0.bag', 'ours8'),
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/ours_9_0.bag', 'ours9'),
        # ('/media/jolee/Samsung_T5/1_bags/sim_exp/ours_10_0.bag', 'ours10'),
        ('/media/jolee/Samsung_T5/1_bags/sim_exp/ours_tracking_0_0.bag', 'ours_tracking'),
        ('/media/jolee/Samsung_T5/1_bags/sim_exp/ours_tracking_1_0.bag', 'ours_tracking2'),
        ('/media/jolee/Samsung_T5/1_bags/sim_exp/art_tracking_0.bag', 'art_tracking1'),
        ('/media/jolee/Samsung_T5/1_bags/sim_exp/art_tracking_1_0.bag', 'art_tracking2'),
        ('/media/jolee/Samsung_T5/1_bags/sim_exp/art_tracking_2_0.bag', 'art_tracking3'),

    ]

    for bag in bag_paths:
        extract_data_from_bag(bag)