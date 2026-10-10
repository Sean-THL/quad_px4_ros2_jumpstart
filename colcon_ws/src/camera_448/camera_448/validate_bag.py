import argparse
from pathlib import Path

import yaml

from camera_448.flight_topics import (
    ALL_RECORD_TOPICS, DEPTH_COMPRESSED_TOPIC, DEPTH_RAW_TOPIC,
    RGB_COMPRESSED_TOPIC, RGB_RAW_TOPIC)


REQUIRED_TOPICS = (
    '/camera_448/color/camera_info',
    '/camera_448/aligned_depth_to_color/camera_info',
    '/vicon/px4_201/px4_201',
    '/px4_201/fmu/out/vehicle_local_position',
    '/px4_201/fmu/in/trajectory_setpoint',
    '/tf_static',
)


def load_counts(bag_directory):
    metadata_path = Path(bag_directory).expanduser().resolve() / 'metadata.yaml'
    if not metadata_path.is_file():
        raise FileNotFoundError(f'No metadata.yaml in {metadata_path.parent}')
    metadata = yaml.safe_load(metadata_path.read_text())
    topics = metadata['rosbag2_bagfile_information']['topics_with_message_count']
    return {
        item['topic_metadata']['name']: int(item['message_count'])
        for item in topics
    }


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Verify that a flight bag contains the visualization data')
    parser.add_argument('bag_directory')
    parsed = parser.parse_args(args)

    try:
        counts = load_counts(parsed.bag_directory)
    except (FileNotFoundError, KeyError, TypeError, yaml.YAMLError) as error:
        print(f'INVALID BAG: {error}')
        return 2

    print('Flight bag topic counts:')
    for topic in ALL_RECORD_TOPICS:
        print(f'{counts.get(topic, 0):8d}  {topic}')

    missing = [topic for topic in REQUIRED_TOPICS if counts.get(topic, 0) <= 0]
    missing_groups = []
    if not any(counts.get(topic, 0) > 0 for topic in
               (RGB_RAW_TOPIC, RGB_COMPRESSED_TOPIC)):
        missing_groups.append('RGB image (raw or compressed)')
    if not any(counts.get(topic, 0) > 0 for topic in
               (DEPTH_RAW_TOPIC, DEPTH_COMPRESSED_TOPIC)):
        missing_groups.append('depth image (raw or compressed)')
    if missing or missing_groups:
        print('\nFAILED: required topics with no messages:')
        for topic in missing:
            print(f'  {topic}')
        for description in missing_groups:
            print(f'  {description}')
        return 1

    if counts.get('/tf', 0) <= 0:
        print('\nNOTE: /tf has no dynamic messages; /tf_static is present.')
    print('\nPASS: RGB, depth, calibration, Vicon, PX4 position, trajectory, and TF are present.')
    return 0
