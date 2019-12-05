# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

import struct

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField


def float_range(min_val, max_val, step):
    val = min_val
    while val <= max_val:
        yield val
        val += step


def xyz_point_fields():
    x_field = PointField()
    x_field.name = 'x'
    x_field.offset = 0
    x_field.datatype = PointField.FLOAT32
    x_field.count = 1

    y_field = PointField()
    y_field.name = 'y'
    y_field.offset = 4
    y_field.datatype = PointField.FLOAT32
    y_field.count = 1

    z_field = PointField()
    z_field.name = 'z'
    z_field.offset = 8
    z_field.datatype = PointField.FLOAT32
    z_field.count = 1

    return (x_field, y_field, z_field)


def uniform_cloud(xs=(-1.0, 1.0), ys=(-1.0, 1.0), zs=(-1.0, 1.0), spacing=0.1):
    msg = PointCloud2()
    msg.is_dense = True
    msg.is_bigendian = True
    # Unordered point cloud
    msg.height = 1
    msg.point_step = 12  # 32bit float x, y, and z

    msg.fields.extend(xyz_point_fields())

    min_x, max_x = xs
    min_y, max_y = ys
    min_z, max_z = zs

    for x in float_range(min_x, max_x, spacing):
        for y in float_range(min_y, max_y, spacing):
            for z in float_range(min_z, max_z, spacing):
                # Assumes native byte order is bigendian
                # if I use >, rviz shows all points at zero
                # is something assuming padding?
                binary = struct.pack('fff', x, y, z)
                assert len(binary) == msg.point_step
                for b in binary:
                    msg.data.append(b)

    msg.width = round(len(msg.data) / msg.point_step)
    msg.row_step = len(msg.data)

    return msg


class FakePointCloudPublisher(Node):

    def __init__(self):
        super().__init__('fake_point_cloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'cloud', 1)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = uniform_cloud()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    fake_point_cloud_publisher = FakePointCloudPublisher()
    rclpy.spin(fake_point_cloud_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
