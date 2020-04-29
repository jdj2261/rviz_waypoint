#!/usr/bin/env python

import yaml
import rospy
import geometry_msgs.msg as geometry_msgs

class WaypointGenerator(object):

    def __init__(self, filename):
        self._sub_pose = rospy.Subscriber('arrow_pose', geometry_msgs.PoseWithCovarianceStamped, self._process_pose, queue_size=1)
        self._waypoints = []
        self._filename = filename

    def _process_pose(self, msg):

        data = {}
        data['frame_id'] = msg.header.frame_id
        data['pose'] = {}
        data['pose']['position'] = {'x': msg.pose.pose.position.x , 'y': msg.pose.pose.position.y, 'z': 0.0}
        data['pose']['orientation'] = {'x': msg.pose.pose.orientation.x, 'y': msg.pose.pose.orientation.y, 'z': msg.pose.pose.orientation.z, 'w':msg.pose.pose.orientation.w}
        data['name'] = '%s_%s' % (msg.pose.pose.position.x, msg.pose.pose.position.y)

        self._waypoints.append(data)
        rospy.loginfo("Clicked : (%s, %s, %s)" % (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))

    def _write_file(self):
        ways = {}
        ways['waypoints'] = self._waypoints
        with open(self._filename, 'w') as f:
            f.write(yaml.dump(ways, default_flow_style=False))
        rospy.loginfo('file saving..')

    def spin(self):
        rospy.spin()
        self._write_file()


if __name__ == '__main__':

    rospy.init_node('waypoint_generator')
    filename = rospy.get_param('~filename', 'output.txt')

    g = WaypointGenerator(filename)
    rospy.loginfo('Initialized')
    g.spin()
    rospy.loginfo('finished')
