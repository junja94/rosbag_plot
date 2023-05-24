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

    se_topic = '/state_estimator/anymal_wheels_state'
    # actuator_topic = '/anymal_wheels_lowlevel_controller/actuator_readings'
    gps_topic = '/gps/gps'
    perf_topic = '/performance_logger/performance_state'
    command_topic = '/locomotion/vel_command'


    dt = rospy.Duration.from_sec(0.5)
    resample_freq = str(dt.to_sec()) + ' S'

    map_frame = 'map_o3d'

    #for anymal bag
    se_topic = '/state_estimator/anymal_state'
    command_topic = '/twist_mux/twist'
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

    # Extract data from rosbag
    linear_velocities_data = []
    base_positions_data = []
    command_data = DataBuffer('command')
    motion_data = DataBuffer('motion')
    cot_data = DataBuffer('cot')
    joint_data = DataBuffer('joint')

    for topic, msg, t in bag.read_messages(topics=[se_topic, perf_topic, command_topic]):

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

                joint_state = {}
                for joint_id, position in enumerate(joint_position):
                    joint_state["joint_position" + str(joint_id)] = position

                for joint_id, velocity in enumerate(joint_velocity):
                    joint_state["joint_velocity" + str(joint_id)] = velocity

                for joint_id, torque in enumerate(joint_torque):
                    joint_state["joint_torque" + str(joint_id)] = torque

                joint_data.append(msg.header.stamp.to_sec(), joint_state)

        # elif topic == perf_topic:
        #     cot_data.append(msg.header.stamp.to_sec(), {'cot': msg.cost_of_transport})
        elif topic == command_topic:
            command = msg_to_command(msg)
            command_data.append(msg.header.stamp.to_sec(), command)

    # convert data to pandas dataframe
    def generate_df_from_buffer(data_buffer):
        df = pd.DataFrame(data=data_buffer.data, index=data_buffer.indices)
        df.index = pd.to_datetime(df.index, unit='s')
        return df

    command_df = generate_df_from_buffer(command_data)
    command_df = command_df.resample(resample_freq).ffill().dropna(axis=0, how='any')

    print(command_df)

    motion_df = generate_df_from_buffer(motion_data)
    # cot_df = generate_df_from_buffer(cot_data)
    joint_df = generate_df_from_buffer(joint_data)

    def resample_df_with_index(df, resampled_idx):
        df.index = pd.to_datetime(df.index, unit='s')
        df = df.reindex(resampled_idx, method='ffill', tolerance=str(dt * 0.5) + 's')
        return df

    def resample_df_with_index_mean(df, resampled_idx):
        df.index = pd.to_datetime(df.index, unit='s')
        df = df.resample(resample_freq).mean()
        df = df.reindex(resampled_idx, method='nearest')
        return df

    motion_df = resample_df_with_index_mean(motion_df, command_df.index)
    # cot_df = resample_df_with_index_mean(cot_df, command_df.index)
    joint_df = resample_df_with_index_mean(joint_df, command_df.index)

    # df_concat = pd.concat([command_df, motion_df, cot_df, joint_df], axis=1)
    df_concat = pd.concat([command_df, motion_df, joint_df], axis=1)

    # Also add time column.. in a weird way
    df_concat['time'] = df_concat.index.microsecond / 1e6 +\
                        df_concat.index.second + \
                        df_concat.index.minute * 60 + \
                        df_concat.index.hour * 3600
    df_concat['time'] = df_concat['time'] - df_concat['time'][0]

    motion_csv_path = os.path.join(save_path, 'motion.csv')
    df_concat.to_csv(motion_csv_path, index=False)

    # Save the GPX file
    gpx = gpxpy.gpx.GPX()
    track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(track)
    # Create a track segment and add it to the track
    segment = gpxpy.gpx.GPXTrackSegment()
    track.segments.append(segment)

    cov_threshold = 10.0
    # Add each GPS point to the track segment
    for topic, msg, t in bag.read_messages(topics=[gps_topic]):
        if msg.position_covariance[0] > cov_threshold and msg.position_covariance[4] > cov_threshold:
            continue
        if msg.status.satellites_visible < 4:
            continue
        point_time = datetime.fromtimestamp(t.to_sec())
        point = gpxpy.gpx.GPXTrackPoint(latitude=msg.latitude, longitude=msg.longitude, elevation=msg.altitude,
                                        time=point_time)
        segment.points.append(point)

    gpx_file_path = os.path.join(save_path, 'data.gpx')

    with open(gpx_file_path, "w") as f:
        f.write(gpx.to_xml())

    print("GPX file saved to:", gpx_file_path)

    bag.close()

if __name__ == '__main__':

    # The bag file should be in the same directory as your terminal
    bag_paths = [
        # ("/home/jolee/Documents/bags/0421_glatt/mission2/2023-04-21-17-40-58_anymal-chimera_mission.bag", '0421_mission2'),
        # ("/home/jolee/Documents/bags/0421_glatt/mission3/2023-04-21-18-01-30_anymal-chimera_mission.bag", '0421_mission3'),
        # ("/home/jolee/Documents/bags/0427_glatt/mission1/2023-04-27-15-42-50_anymal-chimera_mission.bag", '0427_mission1'),
        # ("/home/jolee/Documents/bags/0427_glatt/mission2/2023-04-27-16-18-06_anymal-chimera_mission.bag", '0427_mission2'),
        # ("/home/jolee/Documents/bags/0504_glatt_good/2023-05-04-16-12-59_mission.bag", '0504_mission')
        # ("/home/jolee/Downloads/2023-05-22-13-17-23_anymal-chimera_mission.bag", '0522_mission'),
        ("/home/jolee/Downloads/data_2022-07-31-19-34-26.bag", 'anymal_c'),
    ]

    for bag in bag_paths:
        extract_data_from_bag(bag)