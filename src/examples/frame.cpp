/*
 * @file frame.cpp
 * @Basic framework file
 */

#include <ros/ros.h>  //Include the ROS basics, it allows us to make our nodes and similar stuff

//Include the msg types we need for the program
#include <mavros_msgs/CommandBool.h>     //Msg for arming_client ServiceClient
#include <mavros_msgs/SetMode.h>         //Msg for set_mode_client ServiceClient
#include <mavros_msgs/State.h>           //Msg for state_sub Subscriber
#include <mavros_msgs/AttitudeTarget.h>  //Msg for att_pub Publisher
#include <sensor_msgs/Range.h>           //Msg for rng_sub Subscriber

//Set some global messages for our subscribers to store data in
mavros_msgs::State current_state;
sensor_msgs::Range rngfnd;

//Create a function that is triggered by our state_sub subscriber
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    //Store our state data in our global variable for access in other places
    current_state = *msg;
}

//Create a function that is triggered by our rng_sub subscriber
void rng_cb(const sensor_msgs::Range::ConstPtr& msg){
    //Store our range data in our global variable
    rngfnd = *msg;
}

//Start of our main program
int main(int argc, char **argv)
{
    //These lines create a node with the name offb_node that can be access from the handle nh
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //These are our subscribers, they listen to the given ROS topics and when they hear a 
    //  message on their topic they run their respective function
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ros::Subscriber   rng_sub = nh.subscribe<sensor_msgs::Range>("mavros/rangefinder/rangefinder",1,rng_cb);

    //These are our publishers, they are called as functions and publish messages to a specific topic
    ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);

    //These are our Service Clients, they operate on call and response for important things such as arming the quad
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //The publishing rate of setpoints, must be higher than 2Hz
    ros::Rate rate(20.0);

    //Wait for the quad to come online and connect
    while(ros::ok() && current_state.connected){
        //This command refreshes all publishers, subscribers, and other callbacks
        ros::spinOnce();
        rate.sleep();
    }

    //This is an example of defining an attitude message
    mavros_msgs::AttitudeTarget att;
    att.orientation.x=0;
    att.orientation.y=0;
    att.orientation.z=0;
    att.orientation.w=0;
    att.body_rate.x=0;
    att.body_rate.y=0;
    att.body_rate.z=0;
    att.thrust=.55;
    

    //Send a few setpoints before we start up
    for(int i = 100; ros::ok() && i > 0; --i){
	   //This is an example of publishing a message
       att_pub.publish(att);
       ros::spinOnce();
       rate.sleep();
    }

    //Here we have our Mode message definitions
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED_NOGPS";

    mavros_msgs::SetMode alt_set_mode;
    alt_set_mode.request.custom_mode = "LAND";

    //Here we define our arm command msg
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //This is intialized so we don't overload the FC
    ros::Time last_request = ros::Time::now();

    //The meat of the program, when everything else has been initalized we move on to here
    while(ros::ok()){
        //Here is where we check to see if we're supposed to switch into offboard mode
        if( current_state.mode == "ACRO" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            //Since this is a service client call we get either a true or a false in return depending on whether it succeeded
            if( set_mode_client.call(offb_set_mode)){
                ROS_INFO("Offboard enabled");
            }
            //reset our last request time
            last_request = ros::Time::now();
        } else {
            //If we've switched into offboard mode and we aren't armed we try to arm
            if( !current_state.armed && current_state.mode == "GUIDED_NOGPS" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
    //This is where we continue to send setpoints, realistically this should be
    //  more complex but because we only took off and landed we just kept sending the same waypoint
	att_pub.publish(att);
 
        //If we're in offboard mode and we've made it above our desired
        //  altitude as reported by the lidar we move to landing mode
        if(rngfnd.range>1.5 && current_state.mode=="GUIDED_NOGPS"){
            if(set_mode_client.call(alt_set_mode)){
                ROS_INFO("land");
            }
        }

        //Refresh our subs and pubs every time so we have updated data
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}