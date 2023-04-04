import sys
import os
import time
import threading

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check


def angular_action_movement(base, joint_angles):

    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()

    # move to specified location
    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = joint_angles[joint_id]

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles
        print("joint_angle: ", joint_angle)

    if finished:
        print("Angular movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def cartesian_action_movement(base, base_cyclic):

    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x          # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y - 0.1    # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z - 0.2    # (meters)
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x # (degrees)
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y # (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def GripperCommand(base, target_position):

    # Create the GripperCommand we will send
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    # Close the gripper with position increments
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    finger.finger_identifier = 1
    finger.value = target_position
    print("Going to position {:0.2f}...".format(finger.value))
    base.SendGripperCommand(gripper_command)

    return True


class GripperCommandExample:
    def __init__(self, router, proportional_gain = 2.0):

        self.proportional_gain = proportional_gain
        self.router = router

        # Create base client using TCP router
        self.base = BaseClient(self.router)

    def ExampleSendGripperCommands(self, target_position):

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.finger_identifier = 1
        finger.value = target_position
        print("Going to position {:0.2f}...".format(finger.value))
        self.base.SendGripperCommand(gripper_command)

        return True

        # # Set speed to open gripper
        # print ("Opening gripper using speed command...")
        # gripper_command.mode = Base_pb2.GRIPPER_SPEED
        # finger.value = 0.1
        # self.base.SendGripperCommand(gripper_command)
        # gripper_request = Base_pb2.GripperRequest()

        # # Wait for reported position to be opened
        # gripper_request.mode = Base_pb2.GRIPPER_POSITION
        # while True:
        #     gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
        #     if len (gripper_measure.finger):
        #         print("Current position is : {0}".format(gripper_measure.finger[0].value))
        #         if gripper_measure.finger[0].value < 0.01:
        #             break
        #     else: # Else, no finger present in answer, end loop
        #         break

        # # Set speed to close gripper
        # print ("Closing gripper using speed command...")
        # gripper_command.mode = Base_pb2.GRIPPER_SPEED
        # finger.value = -0.1
        # self.base.SendGripperCommand(gripper_command)

        # # Wait for reported speed to be 0
        # gripper_request.mode = Base_pb2.GRIPPER_SPEED
        # while True:
        #     gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
        #     if len (gripper_measure.finger):
        #         print("Current speed is : {0}".format(gripper_measure.finger[0].value))
        #         if gripper_measure.finger[0].value == 0.0:
        #             break
        #     else: # Else, no finger present in answer, end loop
        #         break


class GripperFeedback:
    def __init__(self, base, base_cyclic, proportional_gain = 2.0, force_min = 10, force_max = 30):
        """
            GripperFeedback class constructor.
            Inputs:
                kortex_api.RouterClient router:            TCP router
                kortex_api.RouterClient router_real_time:  Real-time UDP router
                float proportional_gain: Proportional gain used in control loop (default value is 2.0)
            Outputs:
                None
            Notes:
                - Actuators and gripper initial position are retrieved to set initial positions
                - Actuator and gripper cyclic command objects are created in constructor. Their
                  references are used to update position and speed.
        """

        self.proportional_gain = proportional_gain

        ###########################################################################################
        # UDP and TCP sessions are used in this example.
        # TCP is used to perform the change of servoing mode
        # UDP is used for cyclic commands.
        #
        # 2 sessions have to be created: 1 for TCP and 1 for UDP
        ###########################################################################################

        # Create base client using TCP router
        self.base = base

        # Create base cyclic client using UDP router.
        self.base_cyclic = base_cyclic

        # Create base cyclic command object.
        self.base_command = BaseCyclic_pb2.Command()
        self.base_command.frame_id = 0
        self.base_command.interconnect.command_id.identifier = 0
        self.base_command.interconnect.gripper_command.command_id.identifier = 0

        # Add motor command to interconnect's cyclic
        self.motorcmd = self.base_command.interconnect.gripper_command.motor_cmd.add()

        # Set gripper's initial position velocity and force
        base_feedback = self.base_cyclic.RefreshFeedback()
        self.motorcmd.position = base_feedback.interconnect.gripper_feedback.motor[0].position
        self.motorcmd.velocity = 0
        self.motorcmd.force = force_min

        self.force_min = force_min
        self.force_max = force_max

        for actuator in base_feedback.actuators:
            self.actuator_command = self.base_command.actuators.add()
            self.actuator_command.position = actuator.position
            self.actuator_command.velocity = 0.0
            self.actuator_command.torque_joint = 0.0
            self.actuator_command.command_id = 0
            print("Position = ", actuator.position)

        # Save servoing mode before changing it
        self.previous_servoing_mode = self.base.GetServoingMode()

        # Set base in low level servoing mode
        servoing_mode_info = Base_pb2.ServoingModeInformation()
        servoing_mode_info.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
        self.base.SetServoingMode(servoing_mode_info)

    def Cleanup(self):
        """
            Restore arm's servoing mode to the one that
            was effective before running the example.
            Inputs:
                None
            Outputs:
                None
            Notes:
                None
        """
        # Restore servoing mode to the one that was in use before running the example
        self.base.SetServoingMode(self.previous_servoing_mode)


    def grip(self, target_position):
        if target_position > 100.0:
            target_position = 100.0
        if target_position < 0.0:
            target_position = 0.0
        current_force = self.force_min
        self.motorcmd.position = target_position
        # self.motorcmd.force = self.force_max

        return True

    def Goto(self, target_position):
        """
            Position gripper to a requested target position using a simple
            proportional feedback loop which changes torque according to error
            between target position and current gripper position
            Inputs:
                float target_position: position (0% - 100%) to send gripper to.
            Outputs:
                Returns True if gripper was positionned successfully, returns False
                otherwise.
            Notes:
                - This function blocks until position is reached.
                - If target position exceeds 100.0, its value is changed to 100.0.
                - If target position is below 0.0, its value is set to 0.0.
        """
        if target_position > 100.0:
            target_position = 100.0
        if target_position < 0.0:
            target_position = 0.0
        while True:
            try:
                base_feedback = self.base_cyclic.Refresh(self.base_command)

                # Calculate speed according to position error (target position VS current position)
                position_error = target_position - base_feedback.interconnect.gripper_feedback.motor[0].position
                print("target pos:", target_position)

                # If positional error is small, stop gripper
                if abs(position_error) < 1.5:
                    position_error = 0
                    self.motorcmd.velocity = 0
                    self.base_cyclic.Refresh(self.base_command)
                    return True
                else:
                    self.motorcmd.velocity = self.proportional_gain * abs(position_error)
                    if self.motorcmd.velocity > 100.0:
                        self.motorcmd.velocity = 100.0
                    self.motorcmd.position = target_position

            except Exception as e:
                print(str(e))
                return False
            time.sleep(0.001)
        return True
