#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
import argparse
import json

class PoseDumper:
    def __init__(self, models, output_filename):
        self.models_names = self.get_models_name_list(models)
        self.output_filename = output_filename
        self.result = []
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.states_callback)

    def get_models_name_list(self,names):
        return names.split('/')

    def save_poses(self):
        if self.output_filename:
            out_file = open(self.output_filename, "w")
            json.dump(self.result, out_file)
            rospy.logerr(out_file)
            out_file.close()

    def get_pose_dictionary(self, pose):
        dict = {
            "position": {"x":pose.position.x, "y":pose.position.y, "z":pose.position.z},
            "orientation": {"x":pose.orientation.x, "y":pose.orientation.y, "z":pose.orientation.z, "w":pose.orientation.w}
        }
        return dict

    def states_callback(self, msg):
        poses = {}
        for n in self.models_names:
            index =  msg.name.index(n) if n in msg.name else -1
            if index > -1:
                poses[n] = self.get_pose_dictionary(msg.pose[index])
        self.result.append(poses)

if __name__ == '__main__':
    rospy.init_node('actor_pose_dump_node', anonymous=True)

    parser = argparse.ArgumentParser(description='Dump poses of models from gazebo to files')
    parser.add_argument('--names',default="", type=str, help='List of models names in gazebo world, separated by / , eg. car/school')
    parser.add_argument('--output_filename',default="", type=str, help='Output file name')

    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    pose_dumper = PoseDumper(args.names,args.output_filename)
    rospy.on_shutdown(pose_dumper.save_poses)
    rospy.spin()