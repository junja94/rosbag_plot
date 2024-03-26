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
    file_path = os.path.join(save_path, 'motion_400hz.csv')

    se_topic = '/state_estimator/anymal_wheels_state'

    dt = rospy.Duration.from_sec(0.005)
    resample_freq = str(dt.to_sec()) + ' S'
    map_frame = "odom"

    bag = rosbag.Bag(bag_path)
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    duration = end_time - start_time
    tf_buffer = tf2_py.BufferCore(rospy.Duration.from_sec(duration))

    # fill tf buffer
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        for transform in msg.transforms:
            tf_buffer.set_transform(transform, 'rosbag')

    # Extract data from rosbag
    linear_velocities_data = []
    base_positions_data = []
    command_data = DataBuffer('command')
    motion_data = DataBuffer('motion')
    cot_data = DataBuffer('cot')
    joint_data = DataBuffer('joint')

    with open(file_path, mode='w') as csv_file:
        fieldnames = ['time', 'linear_x', 'linear_y', 'linear_z', 'joint_position', 'joint_velocity', 'joint_torque']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        t_before = 0
        for topic, msg, t in bag.read_messages(topics=[se_topic]):

            if topic == se_topic:
                lin_vel = msg_to_body_lin_vel(msg)  # in baseframe
                # get base position at current time stamp
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
                    # print(t.to_sec() - t_before)
                    #
                    # t_before = t.to_sec()
    # # convert data to pandas dataframe
    # def generate_df_from_buffer(data_buffer):
    #     df = pd.DataFrame(data=data_buffer.data, index=data_buffer.indices)
    #     df.index = pd.to_datetime(df.index, unit='s')
    #     return df
    #
    #
    # motion_df = generate_df_from_buffer(motion_data)
    # # cot_df = generate_df_from_buffer(cot_data)
    # joint_df = generate_df_from_buffer(joint_data)
    #
    #
    # # df_concat = pd.concat([command_df, motion_df, cot_df, joint_df], axis=1)
    # df_concat = pd.concat([motion_df, joint_df], axis=1)
    #
    # # Also add time column.. in a weird way
    # df_concat['time'] = df_concat.index.microsecond / 1e6 +\
    #                     df_concat.index.second + \
    #                     df_concat.index.minute * 60 + \
    #                     df_concat.index.hour * 3600
    # df_concat['time'] = df_concat['time'] - df_concat['time'][0]
    #
    # motion_csv_path = os.path.join(save_path, 'motion_400hz.csv')
    # df_concat.to_csv(motion_csv_path, index=False)
    #

    bag.close()

if __name__ == '__main__':

    # The bag file should be in the same directory as your terminal
    bag_paths = [
        # ("/media/jolee/Samsung_T5/5_cmdp_writing/data/cmdp_gentler_one_0912/1/2023-03-15-16-23-10_anymal-caiman-lpc_mission_0.bag", '0'),
        # ( "/media/jolee/Samsung_T5/5_cmdp_writing/data/0913/cmdp/1_step_1/2023-09-13-18-01-22_anymal-caiman-lpc_mission_0.bag", "cmdp_step_1"),
        # (        "/media/jolee/Samsung_T5/5_cmdp_writing/data/0913/cmdp/2_step_2/2023-09-13-18-03-12_anymal-caiman-lpc_mission_0.bag","cmdp_step_2"),
        # ( "/media/jolee/Samsung_T5/5_cmdp_writing/data/0913/cmdp/0_side/2023-09-13-17-52-19_anymal-caiman-lpc_mission_0.bag", "cmdp_side_1"),
        # ( "/media/jolee/Samsung_T5/5_cmdp_writing/data/0913/ppo/2023-09-13-20-27-07_anymal-caiman-lpc_mission_0.bag", "ppo_step_1")
        # ( "/media/jolee/Samsung_T5/5_cmdp_writing/data/0913/ppo/2023-09-13-20-28-40_anymal-caiman-lpc_mission_0.bag", "ppo_step_2")
        ( "/media/jolee/Samsung_T5/5_cmdp_writing/data/0913/ppo/2023-09-13-20-30-45_anymal-caiman-lpc_mission_0.bag", "ppo_side_1")

    ]

    for bag in bag_paths:
        extract_data_from_bag(bag)