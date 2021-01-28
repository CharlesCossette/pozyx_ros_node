
import pypozyx
import rospy
from data_collector import DataSource, DataCollector
from std_msgs.msg import String
from time import time


time_ns = lambda: int(round(time() * 1e9))

def findPozyxSerial():
    """
    Automatically checks this computer's serial ports for any pozyx
    devices connected through USB.

    Returns:
        pozyxs: [list] of [pypozyx.PozyxSerial] containing one
            PozyxSerial() object per connected pozyx device.
        pozyx_ids: [list] of IDs corresponding to the above PozyxSerial() list
    """
    # Detect Pozyx device on serial ports, get serial port address.
    pozyx_devices = list()
    for port in pypozyx.get_serial_ports():
        if pypozyx.is_pozyx_port(port):
            pozyx_devices.append(port.device)

    if not pozyx_devices:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()
    else:
        print("Pozyx device(s) detected on serial port(s): " \
              + str(pozyx_devices) + ".")

    # Initialize connection to serial port.
    pozyx_serials = list()
    pozyx_ids = list()
    for serial_port in pozyx_devices:
        pozyx = pypozyx.PozyxSerial(serial_port)
        who_am_i = pypozyx.NetworkID()
        status = pozyx.getNetworkId(who_am_i)
        if status == pypozyx.POZYX_FAILURE:
            print("ERROR: Failed to obtain device ID.")

        pozyx_serials.append(pozyx)
        pozyx_ids.append(who_am_i.id)

    return pozyx_serials, pozyx_ids


class PozyxImuSource(DataSource):
    """
    Create a Pozyx IMU source object which can read and return sensor from the
    pozyx device. Certain readings can be excluded using, as an example,

        source = PozyxImuSource(pozyx, mag = False)
        source = PozyxImuSource(pozyx, mag = False, pres = False)
        source = PozyxImuSource(pozyx, gyro = False)
        source = PozyxImuSource(pozyx, remote_id = 0x6a25)

    This class does NOT record range measurements. See PozyxRangeSource()
    """

    def __init__(self, pozyx, accel=True, gyro=True, mag=True,
                 pres=True, euler=False, quat=False, remote_id=None):
        super().__init__()

        # Settings
        self.record_accel = accel
        self.record_gyro = gyro
        self.record_mag = mag
        self.record_pres = pres
        self.record_euler = euler
        self.record_quat = quat
        self.remote_id = remote_id
        self.time_offset = 0
        # Internal variables
        self.pozyx = pozyx
        who_am_i = pypozyx.NetworkID()
        self.pozyx.getNetworkId(who_am_i)
        if self.remote_id is not None:
            self.id = remote_id
            print('Accessing ' + str(hex(self.id)) + ' remotely from ' \
                  + str(hex(who_am_i.id) + '.'))
        else:
            self.id = who_am_i.id
        print('Initalization complete.')

    def getHeader(self):
        """
        Generates the header for the Pozyx IMU Source.
        """
        header_fields = list()
        header_fields.append('Timestamp (ns) Clock Offset: ' + str(self.time_offset))

        # Generate the header string
        if self.record_accel:
            header_fields.append(str(hex(self.id)) + " Accel_x (mg)")
            header_fields.append(str(hex(self.id)) + " Accel_y (mg)")
            header_fields.append(str(hex(self.id)) + " Accel_z (mg)")

        if self.record_gyro:
            header_fields.append(str(hex(self.id)) + " Gyro_x (deg/s)")
            header_fields.append(str(hex(self.id)) + " Gyro_y (deg/s)")
            header_fields.append(str(hex(self.id)) + " Gyro_z (deg/s)")

        if self.record_mag:
            header_fields.append(str(hex(self.id)) + " Mag_x (uT)")
            header_fields.append(str(hex(self.id)) + " Mag_y (uT)")
            header_fields.append(str(hex(self.id)) + " Mag_z (uT)")

        if self.record_euler:
            header_fields.append(str(hex(self.id)) + " Roll (deg)")
            header_fields.append(str(hex(self.id)) + " Pitch (deg)")
            header_fields.append(str(hex(self.id)) + " Yaw (deg)")

        if self.record_quat:
            header_fields.append(str(hex(self.id)) + " Quat_w")
            header_fields.append(str(hex(self.id)) + " Quat_x")
            header_fields.append(str(hex(self.id)) + " Quat_y")
            header_fields.append(str(hex(self.id)) + " Quat_z")

        if self.record_pres:
            header_fields.append(str(hex(self.id)) + " Pressure (Pa)")

        return header_fields

    def getData(self):
        """
        Reads the Pozyx IMU/mag/barometer and returns the data as a list.
        """

        # Containers for storing the data
        data_values = list()

        # Hold until a new IMU measurement is available.
        self.pozyx.waitForFlagSafe(pypozyx.PozyxBitmasks.INT_MASK_IMU, 0.1)

        # Create datastring
        data_values.append(time_ns() - self.time_offset)
        if self.record_accel:
            accel_data = pypozyx.Acceleration()
            self.pozyx.getAcceleration_mg(accel_data, remote_id=self.remote_id)
            data_values.append(accel_data.x)
            data_values.append(accel_data.y)
            data_values.append(accel_data.z)

        if self.record_gyro:
            gyro_data = pypozyx.AngularVelocity()
            self.pozyx.getAngularVelocity_dps(gyro_data, remote_id=self.remote_id)
            data_values.append(gyro_data.x)
            data_values.append(gyro_data.y)
            data_values.append(gyro_data.z)

        if self.record_mag:
            mag_data = pypozyx.Magnetic()
            self.pozyx.getMagnetic_uT(mag_data, remote_id=self.remote_id)
            data_values.append(mag_data.x)
            data_values.append(mag_data.y)
            data_values.append(mag_data.z)

        if self.record_euler:
            euler_data = pypozyx.EulerAngles()
            self.pozyx.getEulerAngles_deg(euler_data, remote_id=self.remote_id)
            data_values.append(euler_data.roll)
            data_values.append(euler_data.pitch)
            data_values.append(euler_data.heading)

        if self.record_quat:
            quat_data = pypozyx.Quaternion()
            self.pozyx.getQuaternion(quat_data, remote_id=self.remote_id)
            data_values.append(quat_data.w)
            data_values.append(quat_data.x)
            data_values.append(quat_data.y)
            data_values.append(quat_data.z)

        if self.record_pres:
            pres_data = pypozyx.Pressure()
            self.pozyx.getPressure_Pa(pres_data, remote_id=self.remote_id)
            data_values.append(pres_data.value)

        return data_values


class PozyxRangeSource(DataSource):
    """
    Create a Pozyx Range source object to do UWB ranging with other pozyx
    devices. Will automatically detect other pozyx devices within UWB range.

    Args:
        pozyx: [PozyxSerial] object. See findPozyxSerial()

        exclude_ids: [list] of device network ids to exclude from the ranging
            process.

        allow_self_ranging: [bool] toggle to include the above exclude_ids
    """

    def __init__(self, pozyx_serial, exclude_ids=[],
                 allow_self_ranging=True, remote_id=None):
        super().__init__()
        self.pozyx = pozyx_serial
        self.allow_self_ranging = allow_self_ranging
        self.exclude_ids = exclude_ids
        self.remote_id = remote_id
        self.time_offset = 0

        who_am_i = pypozyx.NetworkID()
        self.pozyx.getNetworkId(who_am_i)
        if self.remote_id is not None:
            self.id = remote_id
            print('Accessing ' + str(hex(self.id)) + ' remotely from ' \
                  + str(hex(who_am_i.id) + '.'))
        else:
            self.id = who_am_i.id

        self.device_list = self.findNeighbors()
        self._neighbor_to_range = 0
        self._number_of_neighbors = len(self.device_list.data)

    def findNeighbors(self):
        """
        Automatically discovers any pozyx anchor or tag devices within UWB range.

        Returns:
            device_lists: [pypozyx.DeviceList] list containing the device IDs
        """

        # Get ID of current pozyx device
        who_am_i = pypozyx.NetworkID()
        self.pozyx.getNetworkId(who_am_i)

        # Discover other pozyx devices
        self.pozyx.clearDevices(remote_id=self.remote_id)
        self.pozyx.doDiscoveryAll(remote_id=self.remote_id)
        device_list_size = pypozyx.SingleRegister()
        self.pozyx.getDeviceListSize(device_list_size, remote_id=self.remote_id)
        device_list = pypozyx.DeviceList(list_size=device_list_size.value)
        self.pozyx.getDeviceIds(device_list, remote_id=self.remote_id)

        # Print device list
        id_string = "Device " + str(hex(self.id)) + " has discovered the following other devices: "
        for id in device_list.data:
            id_string += str(hex(id)) + ", "

        print(id_string)

        if not self.allow_self_ranging:
            print("However, self-ranging is currently deactivated. " \
                  + "No range measurements between devices connected to the same computer ")
        return device_list

    def getHeader(self):
        """
        Creates the header for the Pozyx Range source.
        """
        header_fields = list()
        header_fields.append('Timestamp (ns) Clock Offset: ' + str(self.time_offset))
        for id in self.device_list.data:
            header_fields.append(str(hex(self.id)) \
                                 + ' to ' + str(hex(id)) + 'Boot Time Pozyx (ms)')
            header_fields.append(str(hex(self.id)) \
                                 + ' Range to ' + str(hex(id)) + ' (mm)')
            header_fields.append(str(hex(self.id)) \
                                 + ' RSS to ' + str(hex(id)) + ' (dB)')
        return header_fields

    def getData(self):
        """
        Performs ranging with one of its neighbors. A different neighbor is
        selected on each of these function calls. Returns range and RSS data.
        """
        if len(self.device_list.data) != 0:
            # Perform the ranging, exclude self-ranging if user has chosen this.
            device_range = pypozyx.DeviceRange()
            status_range = pypozyx.POZYX_FAILURE
            id = self.device_list.data[self._neighbor_to_range]
            if id in self.exclude_ids and not self.allow_self_ranging:
                pass
            else:
                status_range = self.pozyx.doRanging(id, device_range,
                                                    remote_id=self.remote_id)
                if status_range != pypozyx.POZYX_SUCCESS:
                    # Try ranging a second time, sometimes this occurs
                    # if multiple devices are used.
                    status_range = self.pozyx.doRanging(id, device_range, remote_id=self.remote_id)

            # Put data in list.
            data_values = list()
            data_values.append(time_ns() - self.time_offset)
            data_values += [" "] * 3 * self._neighbor_to_range
            if status_range == pypozyx.POZYX_SUCCESS:
                data_values.append(device_range.timestamp)
                data_values.append(device_range.distance)
                data_values.append(device_range.RSS)
            else:
                data_values += [" "] * 3
            data_values += [" "] * 3 * (len(self.device_list.data) - 1 - self._neighbor_to_range)

            # New neighbor for next function call.
            if self._neighbor_to_range == (self._number_of_neighbors - 1):
                self._neighbor_to_range = 0
            else:
                self._neighbor_to_range += 1
        else:
            data_values = list()
            data_values.append(time_ns() - self.time_offset)
        return data_values


class PozyxPositionSource(DataSource):

    def __init__(self, pozyx, anchors):
        super().__init__()

        # positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
        self.algorithm = pypozyx.PozyxConstants.POSITIONING_ALGORITHM_TRACKING
        # self.algorithm = pypozyx.PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
        self.dimension = pypozyx.PozyxConstants.DIMENSION_3D
        self.time_offset = 0
        self.pozyx = pozyx
        who_am_i = pypozyx.NetworkID()
        self.pozyx.getNetworkId(who_am_i)
        self.id = who_am_i.id
        self.rangeOnceToAll()
        self.setAnchorsManual(anchors)
        print('Initalization complete.')

    def rangeOnceToAll(self):
        """
        Automatically discovers any pozyx anchor or tag devices within UWB range,
        and ranges to them. Apparently it helps to range ones to all the other
        devices before starting positions. Produces a more consistent position
        stream.

        Returns:
            device_lists: [pypozyx.DeviceList] list containing the device IDs
        """

        # Get ID of current pozyx device
        who_am_i = pypozyx.NetworkID()
        self.pozyx.getNetworkId(who_am_i)

        # Discover other pozyx devices
        self.pozyx.clearDevices()
        self.pozyx.doDiscoveryAll()
        device_list_size = pypozyx.SingleRegister()
        self.pozyx.getDeviceListSize(device_list_size)
        device_list = pypozyx.DeviceList(list_size=device_list_size.value)
        self.pozyx.getDeviceIds(device_list, )

        # Print device list
        id_string = "Device " + str(hex(self.id)) + " has discovered the following other devices: "
        for id in device_list.data:
            id_string += str(hex(id)) + ", "

        print(id_string)

        # Perform the ranging, exclude self-ranging if user has chosen this.
        device_range = pypozyx.DeviceRange()
        status_range = pypozyx.POZYX_FAILURE
        for id in device_list.data:
            status_range = self.pozyx.doRanging(id, device_range)
        if status_range != pypozyx.POZYX_SUCCESS:
            # Try ranging a second time, sometimes this occurs
            # if multiple devices are used.
            status_range = self.pozyx.doRanging(id, device_range)
        print(device_range.distance)

    def getHeader(self):
        """
        Creates the header for the pozyx position source.
        """
        header_fields = list()
        header_fields.append('Timestamp (ns) Clock Offset: ' + str(self.time_offset))
        header_fields.append(str(hex(self.id)) + " Pos_x (mm)")
        header_fields.append(str(hex(self.id)) + " Pos_y (mm)")
        header_fields.append(str(hex(self.id)) + " Pos_z (mm)")
        header_fields.append(str(hex(self.id)) + " Pressure (Pa)")
        header_fields.append(str(hex(self.id)) + " Temperature (C)")
        return header_fields

    def setAnchorsManual(self, anchors):
        """
        Adds the manually measured anchors to the Pozyx's device list
        one for one.
        """

        self.pozyx.clearDevices()
        for anchor in anchors:
            status = self.pozyx.addDevice(anchor)
        if len(anchors) > 4:
            status = self.pozyx.setSelectionOfAnchors(pypozyx.PozyxConstants.ANCHOR_SELECT_AUTO, len(anchors))

        return status

    def getData(self):
        """
        Performs the built-in positioning algorithm on the pozyx device and
        returns the data.
        """
        position = pypozyx.Coordinates()
        data_values = list()

        status = self.pozyx.doPositioning(position, self.dimension, self.algorithm)
        data_values.append(time_ns() - self.time_offset)
        if status is pypozyx.POZYX_SUCCESS:
            data_values.append(str(position.x))
            data_values.append(str(position.y))
            data_values.append(str(position.z))
        else:
            data_values += [''] * 3

        pres_data = pypozyx.Pressure()
        self.pozyx.getPressure_Pa(pres_data)
        data_values.append(pres_data.value)

        temp_data = pypozyx.Temperature()
        self.pozyx.getTemperature_c(temp_data)
        data_values.append(temp_data.value)

        return data_values


# if __name__ == "__main__":
#     pozyxs, ids = findPozyxSerial()
#     anchors = [pypozyx.DeviceCoordinates(0x6f4a, 1, pypozyx.Coordinates(2594, -2110, 1845)),
#                pypozyx.DeviceCoordinates(0x6f58, 1, pypozyx.Coordinates(-75, 2116, 149)),
#                pypozyx.DeviceCoordinates(0x6f5f, 1, pypozyx.Coordinates(4731, 2149, 2120)),
#                pypozyx.DeviceCoordinates(0x6f60, 1, pypozyx.Coordinates(-4232, 460, 2400)),
#                pypozyx.DeviceCoordinates(0x6f61, 1, pypozyx.Coordinates(-505, -2080, 474))]
# 
#     imu_source1 = PozyxImuSource(pozyxs[0], gyro=False, mag=False, pres=False, accel=False, euler=True)
#     range_source1 = PozyxRangeSource(pozyxs[0])
#     # imu_source2 = PozyxImuSource(pozyxs[1])
#     # range_source2 = PozyxRangeSource(pozyxs[1],exclude_ids=ids,allow_self_ranging=False)
#     dc = DataCollector(imu_source1, range_source1)
#     """
# 
#     position_source1 = PozyxPositionsSource(pozyxs[0],anchors)
#     position_source2 = PozyxPositionSource(pozyxs[1],anchors)
#     dc = DataCollector(position_source1,position_source2)
#     """

#    dc.record(240, name="positions")  # To stream data to screen and save to a

# Modified tutorial. http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
def talker():
    pozyxs, ids = findPozyxSerial()
    anchors = [pypozyx.DeviceCoordinates(0x6f4a, 1, pypozyx.Coordinates(2594, -2110, 1845)),
               pypozyx.DeviceCoordinates(0x6f58, 1, pypozyx.Coordinates(-75, 2116, 149)),
               pypozyx.DeviceCoordinates(0x6f5f, 1, pypozyx.Coordinates(4731, 2149, 2120)),
               pypozyx.DeviceCoordinates(0x6f60, 1, pypozyx.Coordinates(-4232, 460, 2400)),
               pypozyx.DeviceCoordinates(0x6f61, 1, pypozyx.Coordinates(-505, -2080, 474))]

    imu_source1 = PozyxImuSource(pozyxs[0], gyro=False, mag=False, pres=False, accel=False, euler=True)
    range_source1 = PozyxRangeSource(pozyxs[0])
    dc = DataCollector(imu_source1, range_source1)

    #     position_source1 = PozyxPositionsSource(pozyxs[0],anchors)
    #     position_source2 = PozyxPositionSource(pozyxs[1],anchors)
    #     dc = DataCollector(position_source1,position_sour

    headers = dc.getAllHeaders(dc.data_sources)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    rate = rospy.Rate(10) # 10hz - What does Husky actually use?
    while not rospy.is_shutdown():
        data_values = dc.getAllData(dc.data_sources)
        data_string = ",".join(map(str, data_values))
        # Pass the message using kwargs i.e. pub.publish(message_field_1='foo', message_field_2='bar')
        rospy.loginfo(**dict(zip(headers, data_values)))
        pub.publish(**dict(zip(headers, data_values)))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass