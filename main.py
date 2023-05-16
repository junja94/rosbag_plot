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
    return state_msg.joints.position


def msg_to_joint_velocities(state_msg):
    return state_msg.joints.velocity


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

# Press the green button in the gutter to run the script.
if __name__ == '__main__':


    # The bag file should be in the same directory as your terminal
    bag_path = "/home/jolee/Documents/bags/0421_glatt/mission2/2023-04-21-17-40-58_anymal-chimera_mission.bag"
    # bag_path = "/home/jolee/Documents/bags/0421_glatt/mission3/2023-04-21-18-01-30_anymal-chimera_mission.bag"
    # bag_path = "/home/jolee/Documents/bags/0427_glatt/mission1/2023-04-27-15-42-50_anymal-chimera_mission.bag"
    # bag_path = "/home/jolee/Documents/bags/0427_glatt/mission2/2023-04-27-16-18-06_anymal-chimera_mission.bag"
    # bag_path = "/home/jolee/Documents/bags/0504_glatt_good/2023-05-04-16-12-59_mission.bag"


    save_path = os.getcwd()
    save_path += "/data"

    bag = rosbag.Bag(bag_path)
    se_topic = '/state_estimator/anymal_wheels_state'
    actuator_topic = '/anymal_wheels_lowlevel_controller/actuator_readings'
    gps_topic = '/gps/gps'

    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    duration = end_time - start_time

    tf_buffer = tf2_py.BufferCore(rospy.Duration.from_sec(duration))

    # fill tf buffer
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        for transform in msg.transforms:
            tf_buffer.set_transform(transform, 'rosbag')

    # Extract base velocities and positions
    csv_path = save_path + "/linear_velocities.csv"
    map_frame = 'map_o3d'

    with open(csv_path, mode='w') as csv_file:
        fieldnames = ['time', 'linear_x', 'linear_y', 'linear_z', 'base_x', 'base_y', 'base_z']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        # write linear velocities and corresponding base positions to CSV file
        for topic, msg, t in bag.read_messages(topics=[se_topic]):
            lin_vel = msg_to_body_lin_vel(msg)  # in baseframe
            # get base position at current time stamp
            if tf_buffer.can_transform_core(map_frame, 'base', rospy.Time.from_sec(t.to_sec()))[0]:
                base_pos = tf_buffer.lookup_transform_core(map_frame, 'base', rospy.Time.from_sec(t.to_sec()))
                writer.writerow({
                    'time': t.to_sec(),
                    'linear_x': lin_vel[0],
                    'linear_y': lin_vel[1],
                    'linear_z': lin_vel[2],
                    'base_x': base_pos.transform.translation.x,
                    'base_y': base_pos.transform.translation.y,
                    'base_z': base_pos.transform.translation.z
                })

    print("Linear velocities and corresponding base positions saved to {}".format(csv_path))


    # Save GPX file
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

    # Save the GPX file
    gpx_file_path = save_path + "/data.gpx"

    with open(gpx_file_path, "w") as f:
        f.write(gpx.to_xml())

    # performance topics:

    perf_topic = '/performance_logger/performance_state'
    for topic, msg, t in bag.read_messages(topics=[perf_topic]):
        print(msg)

    bag.close()