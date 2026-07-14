# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Provides a driver for NMEA GNSS devices."""

import math

import rospy

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped, QuaternionStamped, Vector3Stamped
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

try:
    from gnss_comm.msg import GnssPVTSolnMsg
except ImportError:
    GnssPVTSolnMsg = None

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser


class RosNMEADriver(object):
    """ROS driver for NMEA GNSS devices."""

    def __init__(self):
        """Initialize the ROS NMEA driver.

        :ROS Publishers:
            - NavSatFix publisher on the 'fix' channel.
            - TwistStamped publisher on the 'vel' channel.
            - QuaternionStamped publisher on the 'heading' channel.
            - TimeReference publisher on the 'time_reference' channel.

        :ROS Parameters:
            - ~time_ref_source (str)
                The name of the source in published TimeReference messages. (default None)
            - ~useRMC (bool)
                If true, use RMC NMEA messages. If false, use GGA and VTG messages. (default False)
            - ~epe_quality0 (float)
                Value to use for default EPE quality for fix type 0. (default 1000000)
            - ~epe_quality1 (float)
                Value to use for default EPE quality for fix type 1. (default 4.0)
            - ~epe_quality2 (float)
                Value to use for default EPE quality for fix type 2. (default (0.1)
            - ~epe_quality4 (float)
                Value to use for default EPE quality for fix type 4. (default 0.02)
            - ~epe_quality5 (float)
                Value to use for default EPE quality for fix type 5. (default 4.0)
            - ~epe_quality9 (float)
                Value to use for default EPE quality for fix type 9. (default 3.0)
        """
        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)
        self.vel_pub = rospy.Publisher('vel', TwistStamped, queue_size=1)
        self.heading_pub = rospy.Publisher(
            'heading', QuaternionStamped, queue_size=1)

        self.uniheading_pub = rospy.Publisher(
            rospy.get_param('~uniheading_topic', 'uniheading'),
            Vector3Stamped, queue_size=10)
        self.uniheading_std_pub = rospy.Publisher(
            rospy.get_param('~uniheading_std_topic', 'uniheading_std'),
            Vector3Stamped, queue_size=10)
        self.uniheading_status_pub = rospy.Publisher(
            rospy.get_param('~uniheading_status_topic', 'uniheading_status'),
            String, queue_size=10)

        self.publish_gnss_pvt = rospy.get_param('~publish_gnss_pvt', True)
        self.gnss_pvt_topic = rospy.get_param('~gnss_pvt_topic', 'receiver_pvt')
        self.gnss_pvt_pub = None
        if self.publish_gnss_pvt:
            if GnssPVTSolnMsg is None:
                rospy.logwarn("~publish_gnss_pvt is true, but gnss_comm/GnssPVTSolnMsg "
                              "cannot be imported. Disable it or add gnss_comm to the workspace.")
            else:
                self.gnss_pvt_pub = rospy.Publisher(
                    self.gnss_pvt_topic, GnssPVTSolnMsg, queue_size=100)

        self.use_GNSS_time = rospy.get_param('~use_GNSS_time', False)
        if not self.use_GNSS_time:
            self.time_ref_pub = rospy.Publisher(
                'time_reference', TimeReference, queue_size=1)

        self.time_ref_source = rospy.get_param('~time_ref_source', None)
        self.use_RMC = rospy.get_param('~useRMC', False)
        self.valid_fix = False

        # epe = estimated position error
        self.default_epe_quality0 = rospy.get_param('~epe_quality0', 1000000)
        self.default_epe_quality1 = rospy.get_param('~epe_quality1', 4.0)
        self.default_epe_quality2 = rospy.get_param('~epe_quality2', 0.1)
        self.default_epe_quality4 = rospy.get_param('~epe_quality4', 0.02)
        self.default_epe_quality5 = rospy.get_param('~epe_quality5', 4.0)
        self.default_epe_quality9 = rospy.get_param('~epe_quality9', 3.0)
        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        """Public method to provide a new NMEA sentence to the driver.

        Args:
            nmea_string (str): NMEA sentence in string form.
            frame_id (str): TF frame ID of the GPS receiver.
            timestamp(rospy.Time, optional): Time the sentence was received.
                If timestamp is not specified, the current time is used.

        Returns:
            bool: True if the NMEA string is successfully processed, False if there is an error.
        """
        if nmea_string.startswith('#'):
            return self.add_unicore_sentence(nmea_string, frame_id, timestamp)

        if not check_nmea_checksum(nmea_string):
            rospy.logwarn("Received a sentence with an invalid checksum. " +
                          "Sentence was: %s" % repr(nmea_string))
            return False

        parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(
            nmea_string)
        if not parsed_sentence:
            rospy.logdebug(
                "Failed to parse NMEA sentence. Sentence was: %s" %
                nmea_string)
            return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()
        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        if not self.use_GNSS_time:
            current_time_ref = TimeReference()
            current_time_ref.header.stamp = current_time
            current_time_ref.header.frame_id = frame_id
            if self.time_ref_source:
                current_time_ref.source = self.time_ref_source
            else:
                current_time_ref.source = frame_id

        if not self.use_RMC and 'GGA' in parsed_sentence:

            current_fix.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            data = parsed_sentence['GGA']
            # rospy.logwarn(data)

            if self.use_GNSS_time:
                if math.isnan(data['utc_time'][0]):
                    rospy.logwarn("Time in the NMEA sentence is NOT valid")
                    return False
                current_fix.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])

            fix_type = data['fix_type']
            if not (fix_type in self.gps_qualities):
                fix_type = -1
            gps_qual = self.gps_qualities[fix_type]
            default_epe = gps_qual[0]
            current_fix.status.status = gps_qual[1]
            current_fix.position_covariance_type = gps_qual[2]

            self.valid_fix = (fix_type > 0)

            current_fix.status.service = NavSatStatus.SERVICE_GPS

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            # use default epe std_dev unless we've received a GST sentence with
            # epes
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            hdop = data['hdop']
            current_fix.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            current_fix.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            current_fix.position_covariance[8] = (
                2 * hdop * self.alt_std_dev) ** 2  # FIXME

            self.fix_pub.publish(current_fix)

            if not (math.isnan(data['utc_time'][0]) or self.use_GNSS_time):
                current_time_ref.time_ref = rospy.Time(
                    data['utc_time'][0], data['utc_time'][1])
                self.last_valid_fix_time = current_time_ref
                self.time_ref_pub.publish(current_time_ref)

        elif not self.use_RMC and 'VTG' in parsed_sentence:
            data = parsed_sentence['VTG']
            # Only report VTG data when you've received a valid GGA fix as
            # well.
            if self.valid_fix:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
                

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            if self.use_GNSS_time:
                if math.isnan(data['utc_time'][0]):
                    rospy.logwarn("Time in the NMEA sentence is NOT valid")
                    return False
                current_fix.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(current_fix)

                if not (math.isnan(data['utc_time'][0]) or self.use_GNSS_time):
                    current_time_ref.time_ref = rospy.Time(
                        data['utc_time'][0], data['utc_time'][1])
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide
            # it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * \
                    math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * \
                    math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
        elif 'GST' in parsed_sentence:
            data = parsed_sentence['GST']

            # Use receiver-provided error estimate if available
            self.using_receiver_epe = True
            self.lon_std_dev = data['lon_std_dev']
            self.lat_std_dev = data['lat_std_dev']
            self.alt_std_dev = data['alt_std_dev']
        elif 'HDT' in parsed_sentence:
            data = parsed_sentence['HDT']
            if data['heading'] is not None:
                current_heading = QuaternionStamped()
                current_heading.header.stamp = current_time
                current_heading.header.frame_id = frame_id
                q = quaternion_from_euler(0, 0, math.radians(data['heading']))
                current_heading.quaternion.x = q[0]
                current_heading.quaternion.y = q[1]
                current_heading.quaternion.z = q[2]
                current_heading.quaternion.w = q[3]
                self.heading_pub.publish(current_heading)
            else:
                rospy.logwarn(data)

        else:
            return False

    def add_unicore_sentence(self, unicore_string, frame_id, timestamp=None):
        """Parse  R1 ASCII logs.
        """
        parsed = libnmea_navsat_driver.parser.parse_unicore_sentence(unicore_string)
        if not parsed:
            rospy.logdebug("Failed to parse Unicore sentence: %s" % unicore_string)
            return False

        current_time = timestamp if timestamp else rospy.get_rostime()

        if 'PVTSLN' in parsed:
            return self._publish_pvtsln(parsed['PVTSLN'], frame_id, current_time)

        if 'UNIHEADING' in parsed:
            return self._publish_uniheading(parsed['UNIHEADING'], frame_id, current_time)

        return False

    def _publish_pvtsln(self, data, frame_id, current_time):
        # Keep original nmea_navsat_driver outputs useful.
        fix = NavSatFix()
        fix.header.stamp = current_time
        fix.header.frame_id = frame_id
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.status.status = NavSatStatus.STATUS_FIX if data['valid_fix'] else NavSatStatus.STATUS_NO_FIX
        fix.latitude = data['latitude']
        fix.longitude = data['longitude']
        fix.altitude = data['altitude']
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        fix.position_covariance[0] = data['lon_std'] * data['lon_std']
        fix.position_covariance[4] = data['lat_std'] * data['lat_std']
        fix.position_covariance[8] = data['hgt_std'] * data['hgt_std']
        self.fix_pub.publish(fix)

        vel = TwistStamped()
        vel.header.stamp = current_time
        vel.header.frame_id = frame_id
        # ROS ENU convention: x east, y north, z up.
        vel.twist.linear.x = data['vel_e']
        vel.twist.linear.y = data['vel_n']
        vel.twist.linear.z = -data['vel_d']
        self.vel_pub.publish(vel)

        if self.gnss_pvt_pub is not None:
            msg = GnssPVTSolnMsg()
            msg.time.week = int(data['week'])
            msg.time.tow = float(data['tow'])

            # Match the semantics used by gnss_comm ublox_driver as closely as possible.
            msg.fix_type = int(data['fix_type'])
            msg.valid_fix = bool(data['valid_fix'])
            msg.diff_soln = bool(data['diff_soln'])
            msg.carr_soln = int(data['carr_soln'])
            msg.num_sv = int(data['num_sv'])

            msg.latitude = float(data['latitude'])
            msg.longitude = float(data['longitude'])
            msg.altitude = float(data['altitude'])
            msg.height_msl = float(data['height_msl'])

            msg.h_acc = float(data['h_acc'])
            msg.v_acc = float(data['v_acc'])
            msg.p_dop = float('nan')

            msg.vel_n = float(data['vel_n'])
            msg.vel_e = float(data['vel_e'])
            msg.vel_d = float(data['vel_d'])
            msg.vel_acc = float(data['vel_acc'])
            self.gnss_pvt_pub.publish(msg)

        return True

    def _publish_uniheading(self, data, frame_id, current_time):
        """Publish R1 UNIHEADINGA.

        Always publish raw dual-antenna heading data on /uniheading so users can
        monitor receiver status even when the heading solution is invalid. Publish
        /heading as QuaternionStamped only when the solution is valid.
        """
        raw_heading = Vector3Stamped()
        raw_heading.header.stamp = current_time
        raw_heading.header.frame_id = frame_id
        raw_heading.vector.x = float(data['heading'])      # deg
        raw_heading.vector.y = float(data['pitch'])        # deg
        raw_heading.vector.z = float(data['length'])       # m
        self.uniheading_pub.publish(raw_heading)

        raw_std = Vector3Stamped()
        raw_std.header.stamp = current_time
        raw_std.header.frame_id = frame_id
        raw_std.vector.x = float(data['heading_std'])      # deg
        raw_std.vector.y = float(data['pitch_std'])        # deg
        raw_std.vector.z = 0.0
        self.uniheading_std_pub.publish(raw_std)

        status = String()
        status.data = '%s,%s' % (data['sol_stat'], data['pos_type'])
        self.uniheading_status_pub.publish(status)

        if not data['valid_heading']:
            return True

        current_heading = QuaternionStamped()
        current_heading.header.stamp = current_time
        current_heading.header.frame_id = frame_id
        q = quaternion_from_euler(0, 0, math.radians(data['heading']))
        current_heading.quaternion.x = q[0]
        current_heading.quaternion.y = q[1]
        current_heading.quaternion.z = q[2]
        current_heading.quaternion.w = q[3]
        self.heading_pub.publish(current_heading)
        return True


    @staticmethod
    def get_frame_id():
        """Get the TF frame_id.

        Queries rosparam for the ~frame_id param. If a tf_prefix param is set,
        the frame_id is prefixed with the prefix.

        Returns:
            str: The fully-qualified TF frame ID.
        """
        frame_id = rospy.get_param('~frame_id', 'gps')
        # Add the TF prefix
        prefix = ""
        prefix_param = rospy.search_param('tf_prefix')
        if prefix_param:
            prefix = rospy.get_param(prefix_param)
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id
