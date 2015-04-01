#!/usr/bin/env python

import argparse
import sys

import rospy
import baxter_interface
from tf.transformations import euler_from_quaternion
from moveit_commander import conversions
from baxter_core_msgs.srv import ( SolvePositionIK,
                                   SolvePositionIKRequest )
from std_msgs.msg import Header
import numpy as np


class Waypoints(object):
    def __init__(self, limb, speed, accuracy):
        # Create baxter_interface limb instance
        self._arm = limb
        self._limb = baxter_interface.Limb(self._arm)

        # Parameters which will describe joint position moves
        self._speed = speed
        self._accuracy = accuracy

        # Standard pose locations
        self._pose = None
        self._lift_height = 0.30

        # Recorded waypoints
        self._block_locations = list()
        self._placement_location = None

        # Recording state
        self._is_recording = False

        # Verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()

        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # Create Navigator I/O
        self._navigator_io = baxter_interface.Navigator(self._arm)

        # gripper ("left" or "right")
        self._gripper = baxter_interface.Gripper(self._arm)
        # calibrate the gripper
        self._gripper.calibrate()
        # set speed of gripper
        self._gripper.set_velocity(10)
        self._gripper.set_holding_force(5)

    def _save_block_location(self, value):
        """
        Stores joint position waypoints

        Navigator 'OK/Wheel' button callback
        """
        if value:
            print("Waypoint Recorded")
            position = self._limb.endpoint_pose()['position']
            orientation = euler_from_quaternion(self._limb.endpoint_pose()['orientation'])
            pose = (position[0],
                position[1],
                position[2],
                orientation[0],
                orientation[1],
                orientation[2])
            self._block_locations.append(pose)

    def _save_place_location(self, value):
        """
        Stores joint position waypoints

        Navigator 'OK/Wheel' button callback
        """
        if value:
            print("Waypoint Recorded")
            position = self._limb.endpoint_pose()['position']
            orientation = euler_from_quaternion(self._limb.endpoint_pose()['orientation'])
            pose = (position[0],
                position[1],
                position[2],
                orientation[0],
                orientation[1],
                orientation[2])
            self._placement_location = pose

    def _stop_recording(self, value):
        """
        Sets is_recording to false

        Navigator 'Rethink' button callback
        """
        # On navigator Rethink button press, stop recording
        if value:
            self._is_recording = False

    def record_block_locations(self):
        """
        Records joint position waypoints upon each Navigator 'OK/Wheel' button
        press.
        """
        rospy.loginfo("Waypoint Recording Started")
        print("Press Navigator 'OK/Wheel' button to record a new joint "
        "joint position waypoint.")
        print("Press Navigator 'Rethink' button when finished recording "
              "waypoints to begin playback")
        # Connect Navigator I/O signals
        # Navigator scroll wheel button press
        self._navigator_io.button0_changed.connect(self._save_block_location)
        # Navigator Rethink button press
        self._navigator_io.button2_changed.connect(self._stop_recording)

        # Set recording flag
        self._is_recording = True

        # Loop until waypoints are done being recorded ('Rethink' Button Press)
        while self._is_recording and not rospy.is_shutdown():
            rospy.sleep(1.0)

        # We are now done with the navigator I/O signals, disconnecting them
        self._navigator_io.button0_changed.disconnect(self._save_block_location)
        self._navigator_io.button2_changed.disconnect(self._stop_recording)

    def record_placement_location(self):
        """
        Records joint position waypoints upon each Navigator 'OK/Wheel' button
        press.
        """
        rospy.loginfo("Waypoint Recording Started")
        print("Press Navigator 'OK/Wheel' button to record a new joint "
        "joint position waypoint.")
        print("Press Navigator 'Rethink' button when finished recording "
              "waypoints to begin playback")
        # Connect Navigator I/O signals
        # Navigator scroll wheel button press
        self._navigator_io.button0_changed.connect(self._save_place_location)
        # Navigator Rethink button press
        self._navigator_io.button2_changed.connect(self._stop_recording)

        # Set recording flag
        self._is_recording = True

        # Loop until waypoints are done being recorded ('Rethink' Button Press)
        while self._is_recording and not rospy.is_shutdown():
            rospy.sleep(1.0)

        # We are now done with the navigator I/O signals, disconnecting them
        self._navigator_io.button0_changed.disconnect(self._save_place_location)
        self._navigator_io.button2_changed.disconnect(self._stop_recording)

    def playback(self):
        """
        Loops playback of recorded joint position waypoints until program is
        exited
        """
        rospy.sleep(1.0)

        rospy.loginfo("Waypoint Playback Started")
        print("  Press Ctrl-C to stop...")

        # Set joint position speed ratio for execution
        self._limb.set_joint_position_speed(self._speed)
        place_location = list(self._placement_location)
        for cur_block_location in self._block_locations:
            if rospy.is_shutdown():
                break
            self.move_to_and_down(cur_block_location)
            self._gripper.close()
            rospy.sleep(0.5)
            self.move_up()
            self.move_to_and_down(place_location)
            self._gripper.open()
            rospy.sleep(0.5)
            self.move_up()
            # place_location[0] += 0.1

        # Set joint position speed back to default
        self._limb.set_joint_position_speed(self._speed)

    def move_up(self):
        current_pose = self._pose
        rospy.sleep(0.5)
        self._pose = (current_pose[0],
                      current_pose[1],
                      self._lift_height,
                      current_pose[3],
                      current_pose[4],
                      current_pose[5])

        self._limb.set_joint_position_speed(0.1)
        self.baxter_ik_move(self._arm, self._pose)
        self._limb.set_joint_position_speed(self._speed)

    # def move_to_and_down(self, pose):
    #     current_pose = self._pose
    #
    #     self._pose = (pose[0],
    #                   pose[1],
    #                   self._lift_height,
    #                   pose[3],
    #                   pose[4],
    #                   pose[5])
    #
    #     self.baxter_ik_move(self._arm, self._pose)
    #
    #     rospy.sleep(0.5)
    #
    #     self._pose = (pose[0],
    #                   pose[1],
    #                   pose[2],
    #                   pose[3],
    #                   pose[4],
    #                   pose[5])
    #
    #     self._limb.set_joint_position_speed(0.1)
    #     self.baxter_ik_move(self._arm, self._pose)
    #     self._limb.set_joint_position_speed(self._speed)
    #
    #     rospy.sleep(0.5)

    def move_to_and_down(self, pose):
        current_pose = self._pose

        self._pose = (pose[0],
                      pose[1],
                      self._lift_height,
                      pose[3],
                      pose[4],
                      pose[5])

        self.baxter_ik_move(self._arm, self._pose)

        rospy.sleep(2)

        self._limb.set_joint_position_speed(0.1)
        # print "Starting move down..."
        for cur_height in np.logspace(np.log10(0.005), np.log10((self._lift_height-pose[2])/2), 7)[::-1]:
            self._pose = (pose[0],
                          pose[1],
                          cur_height+pose[2],
                          pose[3],
                          pose[4],
                          pose[5])
            self.baxter_ik_move(self._arm, self._pose)

            print self._limb.endpoint_effort()
            if np.abs(self._limb.endpoint_effort()['force'].z) > 10:
                # take force out of limbs
                position = self._limb.endpoint_pose()['position']
                orientation = euler_from_quaternion(self._limb.endpoint_pose()['orientation'])
                self._pose = (position[0],
                position[1],
                position[2]+0.02,
                orientation[0],
                orientation[1],
                orientation[2])
                self.baxter_ik_move(self._arm, self._pose)
                print "Force is greater than acceptable, aborting placement... taking tension out of arms."
                break

        print "Finished move down..."

        self._limb.set_joint_position_speed(self._speed)

        rospy.sleep(0.5)

    def clean_shutdown(self):
        print("\nExiting example...")
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    # move a limb
    def baxter_ik_move(self, arm, rpy_pose):
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + arm + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")

        if ik_response.isValid[0]:
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            self._limb.move_to_joint_positions(limb_joints, timeout=2) # threshold=0.001)
        else:
            # display invalid move message on head display
            # self.splash_screen("Invalid", "move")
            # little point in continuing so exit with error message
            print "requested move =", rpy_pose
            sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")


def main():
    """RSDK Joint Position Waypoints Example

    Records joint positions each time the navigator 'OK/wheel'
    button is pressed.
    Upon pressing the navigator 'Rethink' button, the recorded joint positions
    will begin playing back in a loop.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='limb to record/playback waypoints'
    )
    parser.add_argument(
        '-s', '--speed', default=0.3, type=float,
        help='joint position motion speed ratio [0.0-1.0] (default:= 0.3)'
    )
    parser.add_argument(
        '-a', '--accuracy',
        default=baxter_interface.settings.JOINT_ANGLE_TOLERANCE, type=float,
        help='joint position accuracy (rad) at which waypoints must achieve'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_waypoints_%s" % (args.limb,))

    waypoints = Waypoints(args.limb, args.speed, args.accuracy)

    # Register clean shutdown
    rospy.on_shutdown(waypoints.clean_shutdown)

    # Begin example program
    waypoints.record_block_locations()
    waypoints.record_placement_location()
    waypoints.playback()

if __name__ == '__main__':
    main()