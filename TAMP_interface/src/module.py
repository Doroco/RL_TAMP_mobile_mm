#!/usr/bin/env python
__author__ = "Thomas Rostrup Andersen"
__copyright__ = "Copyright (C) 2016 Thomas Rostrup Andersen"
#__license__ = ""
__version__ = "0.0.2"
__all__ = ['Module']

import rospy
import actionlib
import smach
import smach_ros
import time
import json
from rospy_message_converter import message_converter

from std_msgs.msg import String
from cyborg_controller.msg import StateMachineAction, StateMachineGoal, SystemState

# Define state Module
class Module(smach.State):
    """Module is a state type for SMACH. It uses an action from a server and listens for events."""

    def __init__(self, state_name, actionlib_name, transitions, resources):
        self.state_name = state_name
        self.actionlib_name = actionlib_name
        self.transitions = transitions
        self.resources = resources
        self.active = False
        self.registered_events = []
        self.publisher = rospy.Publisher("/cyborg_controller/state_change", SystemState, queue_size=100)
        self.transitions_publisher = rospy.Publisher("/cyborg_controller/state_transitions", String, queue_size=100)
        smach.State.__init__(self, self.transitions.keys(), input_keys=["input_events", "previous_state", "previous_event"], output_keys=["output_events", "current_state", "current_event"])
        self.subscriber = rospy.Subscriber( "/cyborg_controller/register_event", String, self.register_event, queue_size=100)
        
    # Called once when the goal completes
    def callback_done(self, state, result):
        self.server_state = state
        self.server_result = result
        rospy.logdebug("State named " + self.state_name + " has completed its execution with " + str(state) + " and result " + str(result) + ".")
        self.active = False

    # Called once when the goal becomes active
    def callback_active(self):
        self.active = True
        rospy.logdebug("State named " + self.state_name + " has gone active at client named " + self.actionlib_name + ".")

    # Called every time feedback is received for the goal
    def callback_feedback(self, feedback):
        self.server_feedback = feedback
        rospy.logdebug("State Machine got feedback: " + str(feedback))

    # Called when an event is received. Event is registered in the self.registered_event list
    def register_event(self, data):
        rospy.logdebug("Event received: " + data.data)
        self.registered_events.append(data.data)

    # Inform all nodes about the state change (gate keepers will now check the new resource list on the server)
    def publish_state_change(self):
        msg = SystemState()
        msg.event = self.previous_event
        msg.from_system_state = self.previous_state
        msg.to_system_state = self.state_name
        rospy.logdebug("State changing from " + self.previous_state + " to state " + self.state_name + " because of event named " + self.previous_event)
        self.publisher.publish(msg)

    # Called once every time the state machine enters this state.
    # Gets the userdata, allocate the resources and publishes the state change (to the gate keepers)
    def enter(self, userdata):
        # Get userdata (and information about last state)
        self.registered_events = userdata.input_events # Get the remaning unhandled events from the state machine
        self.previous_state = userdata.previous_state
        self.previous_event = userdata.previous_event
        rospy.set_param("/state_resources", self.resources) # Change the ROS servers resource parameter. List of resources the state (and it nodes) want access to

        self.publish_state_change() # inform all nodes (gate keepers) about the state change (gate keepers will now check the new resource list on the server)
        rospy.logdebug("Executing state: " + self.state_name + ", event list is " + str(self.registered_events))
        self.active = True


    # Called once every time the state machine leaves this state
    def leave(self, event, userdata):
        userdata.output_events = self.registered_events
        userdata.current_state = self.state_name
        userdata.current_event = event
        self.active = False
        return userdata

    # When state is entered, this function is called. It returns the event that leads to the next state.
    def execute(self, userdata):
        self.enter(userdata=userdata)

        # Create a connection to (the State) ROS Node server that provide the state execution (ROS actionlib)
        rospy.logdebug("Server connection is: " + self.actionlib_name)
        client = actionlib.SimpleActionClient(self.actionlib_name, StateMachineAction)
        if not client.wait_for_server(rospy.Duration.from_sec(5.0)):
                rospy.logwarn("ERROR: Unable to connect to state server node for %s." % self.actionlib_name)
                userdata = self.leave(event="aborted", userdata=userdata)
                return "aborted"

        # Create an action goal message and send it
        goal = StateMachineGoal()
        goal.order = "EXECUTE"
        goal.previous_state = self.previous_state
        goal.event = self.previous_event
        goal.current_state = self.state_name
        client.send_goal(goal, self.callback_done, self.callback_active, self.callback_feedback)
        
        #convert state transitions dict to ros message and publish for use in GUI
        transitions = self.transitions
        transitionsStringed = json.dumps(transitions)
        dict_message = {'data': transitionsStringed}
        transitions_msg = message_converter.convert_dictionary_to_ros_message("std_msgs/String", dict_message)
        self.transitions_publisher.publish(transitions_msg)

        # State is blocking until it completes or a valid event trigger a state change
        while (True):
            if not self.active: # aka goal is completed from the self.callback_done function
                event = "succeeded" if self.server_state == 3 else "aborted"
                userdata = self.leave(event=event, userdata=userdata)
                return event

            # Check events
            if self.registered_events:
                event = self.registered_events.pop(0) 
                rospy.logdebug("Handling event: " + event)
                if (self.transitions.keys().count(event) > 0):
                    client.cancel_goal()
                    while (self.active):
                        time.sleep(0.1) # delays for some time(seconds) 
                    userdata = self.leave(event=event, userdata=userdata)
                    return event
            else:
                time.sleep(0.1) # delays for some time(seconds)
