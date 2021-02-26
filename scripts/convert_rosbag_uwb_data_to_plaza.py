from os import times
import rosbag
import rospy

eps = 1e-3


def measurement_is_valid_uwb(topic, msg):
    """Checks to see that is a valid UWB measurement based on topic name and
    distance measurement

    Args:
        topic ([type]): [description]
        msg ([type]): [description]

    Returns:
        [type]: [description]
    """
    data = msg.uwb
    dist = float(data.dist)/1000

    return "/uwb_msg" in topic and dist > eps


def get_uwb_topic_namespace(topic):
    namespace_end_ind = topic.find('uwb_msg')-1
    # ignore first character because is normally a slash
    assert topic[0] == "/", "first character was not a slash"
    namespace = topic[1:namespace_end_ind]
    return namespace


def all_uwb_had_measurements(uwb_measurements):
    for uwb_addr in uwb_measurements.keys():
        if uwb_measurements[uwb_addr] == 0:
            return False
    return True


def print_uwb_measurement_statistics(uwb_measurements):
    print
    for uwb_addr in uwb_measurements.keys():
        print('UWB {} had {} measurements'.format(
            uwb_addr, uwb_measurements[uwb_addr]))
    print


def rosbag_has_uwb_msg(bag):
    topic_info = bag.get_type_and_topic_info()[1]
    topics = topic_info.keys()

    for topic in topics:
        if "/uwb_msg" in topic:
            return True

    return False


def get_robot_id_from_robot_name(name, name_id_map):
    return name_id_map[name]


def get_robot_name_from_uwb_addr(addr, uwb_addr_map):
    return uwb_addr_map[addr]


def get_robot_id_from_uwb_addr(addr, name_id_map, uwb_addr_map):
    name = get_robot_name_from_uwb_addr(addr, uwb_addr_map)
    return get_robot_id_from_robot_name(name, name_id_map)


def debug_uwb_measurements(uwb_bag_path, robots_with_tags, robot_id_map,
                           uwb_addr_name_map):
    """Sanity check all of the UWB measurements by makeing sure that all UWBs
    saw measurements and printing out the number of measurements that each
    UWB recorded

    Args:
        uwb_bag_path ([type]): [description]
        robots_with_tags ([type]): [description]
        robot_id_map ([type]): [description]
        uwb_addr_name_map ([type]): [description]
    """
    uwb_rosbag = rosbag.Bag(uwb_bag_path)

    # variables to track stats on UWB measurements
    tag_robots_seen = set()
    uwb_measurements = {addr: 0 for addr in uwb_addr_name_map.keys()}
    distances_measured = []

    # iterate over all measurements but only focus on the UWB measurements
    for topic, msg, timestamp in uwb_rosbag.read_messages():
        if measurement_is_valid_uwb(topic, msg):
            data = msg.uwb
            addr = data.addr
            dist = float(data.dist)/1000
            robot_name = get_uwb_topic_namespace(topic)
            if dist > eps:
                # make sure that all robots with tags are filled
                tag_robots_seen.add(robot_name)

                # count how often each UWB has measurement
                uwb_measurements[str(addr)] += 1

                # check histogram of distances measured
                distances_measured.append(dist)

            else:
                anchor_robot_name = get_robot_name_from_uwb_addr(
                    addr, uwb_addr_name_map)

                # only print if the near 0 measurement was on the same robot
                if not robot_name == anchor_robot_name:
                    print(
                        "near 0 distance measurement between {} and UWB {}\n".format(
                            robot_name, addr))

    # make sure that all tags attached to robots had measurements
    print_uwb_measurement_statistics(uwb_measurements)
    assert tag_robots_seen == robots_with_tags, "did not see measurements from all tags attached to robots. Check that all were functioning and that the provided list of tag robots is correct"
    assert all_uwb_had_measurements(
        uwb_measurements), "not all UWBs were recorded"


def convert_to_measurement(topic, msg, timestamp, name_id_map, uwb_addr_map):
    """takes in all message information and returns a tuple specifying the
    actual measurement info in form:
        (timestamp, robot_id, anchor_robot_id, dist)

    Args:
        topic ([type]): message topic
        msg ([type]): message data
        timestamp ([type]): message timestamp
        name_id_map (dict): mapping between robot name and robot id
        uwb_addr_map (dict): mapping between UWB measurement and robot name
    """
    robot_name = get_uwb_topic_namespace(topic)
    robot_id = get_robot_id_from_robot_name(robot_name, name_id_map)
    data = msg.uwb
    dist = float(data.dist)/1000
    addr = data.addr
    anchor_robot_id = get_robot_id_from_uwb_addr(
        addr, name_id_map, uwb_addr_map)

    time = timestamp.to_sec()

    return(time, robot_id, anchor_robot_id, dist)


def get_range_measurements(rosbag_path, name_id_map, uwb_addr_map):
    """parses through a rosbag and returns a list of distance measurement
    tuples in the format (timestamp, robot_id1, robot_id2, distance)

    Args:
        rosbag_path (str): the path to the rosbag
        name_id_map (dict): mapping between robot name and robot id
        uwb_addr_map (dict): mapping between UWB measurement and robot name

    Returns:
        List[Tuple]: list of range measurements, format (timestamp, robot_id1,
        robot_id2, distance)
    """
    uwb_rosbag = rosbag.Bag(rosbag_path)
    topic_info = uwb_rosbag.get_type_and_topic_info()[1]
    topics = topic_info.keys()

    # make sure that there are uwb measurements in bag
    assert rosbag_has_uwb_msg(uwb_rosbag), "No UWB messages found in rosbag"

    # build list of measurements
    range_measurements = []
    for topic, msg, timestamp in uwb_rosbag.read_messages():
        if measurement_is_valid_uwb(topic, msg):
            # get measurement tuple and add to list
            measurement = convert_to_measurement(
                topic, msg, timestamp, name_id_map, uwb_addr_map)
            range_measurements.append(measurement)

    return range_measurements


def write_range_measurements_to_file(range_measurements, output_file_path, output_format):
    """writes all range measurements passed in to an output file

    Args:
        range_measurements (List): list of range measurement tuples
        output_file_path (str): output file path
        output_format (str): specifying what format to save the file to

    Raises:
        NotImplementedError: tried to save to file path not supported
    """
    if output_format.lower() == "plaza":
        # execute
        print("Writing range measurements in plaza dataset format to " + output_file_path)

        data_file = open(output_file_path, 'w')

        # measurement format (timestamp, robot_id1, robot_id2, distance)
        for measurement in range_measurements:
            time = measurement[0]
            id1 = measurement[1]
            id2 = measurement[2]
            dist = measurement[3]

            data_file.write('{} {} {} {}\n'.format(
                time, id1, id2, dist))

        data_file.close()

    else:
        raise NotImplementedError


if __name__ == "__main__":
    """Converts a rosbag that contains multirobot UWB ranging data into the
    plaza data format. All of the inputs to this program should be modifiable in
    this main function. Performs some sanity checks on the data processed to make
    sure that the UWBs were all functioning and able to connect with each other.
    """

    # dataset information
    uwb_bag_path = "/home/thumman/Desktop/swarmbot-related/initial_experiment_rosbags/uwb_msg_test.bag"
    output_format = "plaza"
    output_file_path = "/home/thumman/Desktop/swarmbot-related/initial_experiment_rosbags/output.txt"

    # uwb configuration information
    robot_id_map = {"mrg1": 1,
                    "mrg2": 2,
                    "archimedes": 3,
                    "pythagoras": 4,
                    "grace": 5,
                    "susan": 6}
    robots_with_tags = {"mrg1", "archimedes", "pythagoras", "susan"}
    uwb_addr_name_map = {"2ed5": "mrg2",
                         "2f50": "pythagoras",
                         "2faa": "grace",
                         "2fab": "susan",
                         "c50": "mrg1",
                         "2eb2": "mrg2",
                         "2f95": "pythagoras",
                         "971f": "grace",
                         "9631": "mrg2",
                         "9838": "archimedes",
                         "8583": "pythagoras",
                         "2f68": "grace"}

    # get list of range measurement tuples
    # tuple format (timestamp, robot_id1, robot_id2, distance)
    range_measurements = get_range_measurements(
        uwb_bag_path, robot_id_map, uwb_addr_name_map)

    # write all recorded measurement to file
    write_range_measurements_to_file(
        range_measurements, output_file_path, output_format)

    # sanity check all measurements
    debug_uwb_measurements(uwb_bag_path, robots_with_tags,
                           robot_id_map, uwb_addr_name_map)
