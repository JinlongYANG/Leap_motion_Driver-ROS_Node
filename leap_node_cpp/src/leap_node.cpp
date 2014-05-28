/******************************************************************************\
* Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

#include <ros/ros.h>
#include <iostream>

#include <leap_msgs/Leap.h>
#include <leap_msgs/Hand.h>
#include <leap_msgs/Finger.h>
#include <leap_msgs/Tool.h>
#include <leap_msgs/Gesture.h>
#include <leap_msgs/LeapService.h>
#include <tf/transform_datatypes.h>
#include <qt4/Qt/qmutex.h>
#include <Leap.h>
#include <visualization_msgs/Marker.h>

using namespace Leap;

class SampleListener : public Leap::Listener {
public:
    virtual void onInit(const Leap::Controller&);
    virtual void onConnect(const Leap::Controller&);
    virtual void onDisconnect(const Leap::Controller&);
    virtual void onExit(const Leap::Controller&);
    virtual void onFrame(const Leap::Controller&);
    virtual void onFocusGained(const Leap::Controller&);
    virtual void onFocusLost(const Leap::Controller&);
    virtual void onDeviceChange(const Leap::Controller&);
    virtual void onServiceConnect(const Leap::Controller&);
    virtual void onServiceDisconnect(const Leap::Controller&);
    bool leap_service_callback(leap_msgs::LeapServiceRequest &req, leap_msgs::LeapServiceResponse &res);
public:
    ros::NodeHandle nh;
    ros::Publisher leap_msg_pub;
    ros::ServiceServer leap_service;
    ros::Publisher leap_gesture_pub;
    ros::Publisher marker_pub;
    leap_msgs::Leap leap_data_msg;
    leap_msgs::Leap leap_msg_local;
    QMutex mutex;
};

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};

bool SampleListener::leap_service_callback(leap_msgs::LeapServiceRequest &req, leap_msgs::LeapServiceResponse &res)
{
    mutex.lock();
    res.leap_msg = leap_msg_local;
    mutex.unlock();
    return true;
}

void SampleListener::onInit(const Leap::Controller& controller) {
    std::cout << "Initialized" << std::endl;
    leap_msg_pub = nh.advertise<leap_msgs::Leap>("leap_data", 1000);
    leap_gesture_pub = nh.advertise<leap_msgs::Gesture>("leap_gestures", 1000);
    leap_service = nh.advertiseService("leap_service", &SampleListener::leap_service_callback, this);
    marker_pub = nh.advertise<visualization_msgs::Marker>("vis_marker",1000);
    //ros::spinOnce();
    ros::Rate(200);
}

void SampleListener::onConnect(const Leap::Controller& controller) {
    std::cout << "Connected" << std::endl;
    controller.enableGesture(Leap::Gesture::TYPE_CIRCLE);
    controller.enableGesture(Leap::Gesture::TYPE_KEY_TAP);
    controller.enableGesture(Leap::Gesture::TYPE_SCREEN_TAP);
    controller.enableGesture(Leap::Gesture::TYPE_SWIPE);
}

void SampleListener::onDisconnect(const Leap::Controller& controller) {
    //Note: not dispatched when running in a debugger.
    std::cout << "Disconnected" << std::endl;
}

void SampleListener::onExit(const Leap::Controller& controller) {
    std::cout << "Exited" << std::endl;
}

void SampleListener::onFrame(const Leap::Controller& controller) {

    leap_msgs::Leap leap_msg;

    // Get the most recent frame and report some basic information
    const Leap::Frame frame = controller.frame();
    std::cout << "Frame id: " << frame.id()
              << ", timestamp: " << frame.timestamp()
              << ", hands: " << frame.hands().count()
              << ", fingers: " << frame.fingers().count()
              << ", tools: " << frame.tools().count()
              << ", pointables: " << frame.pointables().count()
              << ", gestures: " << frame.gestures().count() << std::endl;

    leap_msg.leap_frame_id = frame.id();
    leap_msg.leap_time_stamp = frame.timestamp();
    leap_msg.hands_count = frame.hands().count();
    leap_msg.fingers_count = frame.fingers().count();
    leap_msg.tools_count = frame.tools().count();
    leap_msg.gestures_count = frame.gestures().count();

    Leap::Matrix tf_matrix;

    if (!frame.hands().isEmpty()) {
        // Get the first hand
        Leap::HandList hands = frame.hands();
//      const Leap::Hand hand = frame.hands()[0];
        for(int hand_id = 0; hand_id < hands.count(); hand_id++)
        {
            leap_msgs::Hand hand_msg;
            hand_msg.id = hands[hand_id].id();

            // Check if the hand has any fingers
            const Leap::FingerList fingers = hands[hand_id].fingers();
            if (!fingers.isEmpty()) {
                // Calculate the hand's average finger tip position
                Leap::Vector avgPos;
                for (int i = 0; i < fingers.count(); ++i) {
                    avgPos += fingers[i].tipPosition();
                    hand_msg.finger_ids.push_back(fingers[i].id());
                }
                avgPos /= (float)fingers.count();
//                std::cout << "Hand has " << fingers.count()
//                          << " fingers, average finger tip position" << avgPos << std::endl;
            }

            const Leap::ToolList tools = hands[hand_id].tools();
            if(!tools.isEmpty())
            {
                for(int i = 0;i < tools.count();i++)
                {
                    hand_msg.tool_ids.push_back(tools[i].id());
                }
            }

            // Get the hand's sphere radius and palm position
//            std::cout << "Hand sphere radius: " << hands[hand_id].sphereRadius()
//                      << " mm, palm position: " << hands[hand_id].palmPosition() << std::endl;

            // Get the hand's normal vector and direction
            const Leap::Vector normal = hands[hand_id].palmNormal();
            const Leap::Vector direction = hands[hand_id].direction();

            // Calculate the hand's pitch, roll, and yaw angles
//            std::cout << "Hand pitch: " << direction.pitch() * Leap::RAD_TO_DEG << " degrees, "
//                      << "roll: " << normal.roll() * Leap::RAD_TO_DEG << " degrees, "
//                      << "yaw: " << direction.yaw() * Leap::RAD_TO_DEG << " degrees" << std::endl;

            hand_msg.pose.position.x = hands[hand_id].palmPosition().x;
            hand_msg.pose.position.y = hands[hand_id].palmPosition().y;
            hand_msg.pose.position.z = hands[hand_id].palmPosition().z;
            tf::Quaternion q_hand;

            q_hand.setRPY(normal.roll(), direction.pitch(), direction.yaw());
            hand_msg.pose.orientation.w = q_hand.getW();
            hand_msg.pose.orientation.x = q_hand.getZ();
            hand_msg.pose.orientation.y = q_hand.getY();
            hand_msg.pose.orientation.z = q_hand.getX();

            // add a coordinatesystem of hand
            Leap::Vector handXbasis = hands[hand_id].palmNormal().cross(hands[hand_id].direction()).normalized();
            Leap::Vector handYbasis = hands[hand_id].direction();
            Leap::Vector handZbasis = -hands[hand_id].palmNormal();
            Leap::Vector hand_position = hands[hand_id].palmPosition();
            hand_msg.xAxis.x = handXbasis.x;
            hand_msg.xAxis.y = handXbasis.y;
            hand_msg.xAxis.z = handXbasis.z;
            hand_msg.yAxis.x = handYbasis.x;
            hand_msg.yAxis.y = handYbasis.y;
            hand_msg.yAxis.z = handYbasis.z;
            hand_msg.zAxis.x = handZbasis.x;
            hand_msg.zAxis.y = handZbasis.y;
            hand_msg.zAxis.z = handZbasis.z;
            tf_matrix = Leap::Matrix(handXbasis,handYbasis,handZbasis,hand_position);
            tf_matrix = tf_matrix.rigidInverse();

            //Add data from Leap v2.0
            Leap::Hand ha; ha.pinchStrength();
            if(hands[hand_id].isRight())
                hand_msg.type = 1;
            else if(hands[hand_id].isLeft())
                hand_msg.type = 2;
            else
                hand_msg.type = -1;
            hand_msg.grab_strenght = hands[hand_id].grabStrength();
            hand_msg.pinch_strenght = hands[hand_id].pinchStrength();
            hand_msg.data_confidence = hands[hand_id].confidence();
            leap_msg.hands.push_back(hand_msg);
        }
    }

    // Fill in tool_msg data
    if(!frame.tools().isEmpty())
    {
        Leap::ToolList tools = frame.tools();
        for(int tool_id = 0;tool_id < tools.count();tool_id++)
        {
            leap_msgs::Tool tool_msg;
            tool_msg.id = tools[tool_id].id();
            tool_msg.length = tools[tool_id].length();
            tool_msg.width = tools[tool_id].width();
            tool_msg.velocity.x = tools[tool_id].tipVelocity().x;
            tool_msg.velocity.y = tools[tool_id].tipVelocity().y;
            tool_msg.velocity.z = tools[tool_id].tipVelocity().z;
            tool_msg.pose.position.x = tools[tool_id].tipPosition().x;
            tool_msg.pose.position.y = tools[tool_id].tipPosition().y;
            tool_msg.pose.position.z = tools[tool_id].tipPosition().z;
            tf::Quaternion q_tool;
            q_tool.setRPY(tools[tool_id].tipPosition().roll(), tools[tool_id].tipPosition().pitch(), tools[tool_id].tipPosition().yaw());
            tool_msg.pose.orientation.w = q_tool.getW();
            tool_msg.pose.orientation.x = q_tool.getX();
            tool_msg.pose.orientation.y = q_tool.getY();
            tool_msg.pose.orientation.z = q_tool.getZ();
            leap_msg.tools.push_back(tool_msg);

        }
    }

    // Fill in finger_msg data
    if(!frame.fingers().isEmpty())
    {
        visualization_msgs::Marker marker_finger, line_strip, trajectory_finger;
        marker_finger.header.frame_id = line_strip.header.frame_id = trajectory_finger.header.frame_id = "map";
        marker_finger.header.stamp = line_strip.header.stamp = trajectory_finger.header.stamp = ros::Time::now();
        marker_finger.ns =  line_strip.ns = trajectory_finger.ns = "finger";
        marker_finger.action = line_strip.action = trajectory_finger.action = visualization_msgs::Marker::ADD;;
        marker_finger.pose.orientation.w = line_strip.pose.orientation.w = 1;
        marker_finger.id = 0;
        marker_finger.type = visualization_msgs::Marker::POINTS;
        marker_finger.scale.x= 10; marker_finger.scale.y= 10;
        marker_finger.color.g = 1.0f; marker_finger.color.a = 1.0;
        line_strip.id = 1;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 1; line_strip.scale.y = 1;
        line_strip.color.b = 1.0; line_strip.color.a = 1.0;
        trajectory_finger.id = 2;
        trajectory_finger.type = visualization_msgs::Marker::LINE_LIST;
        trajectory_finger.scale.x = 1; trajectory_finger.scale.y = 1;
        trajectory_finger.color.r = 1.0; trajectory_finger.color.a = 1.0;


        Leap::FingerList fingers = frame.fingers();
        for(int finger_id = 0;finger_id < fingers.count();finger_id++)
        {
            //const Finger finger = fingers[finger_id];
            leap_msgs::Finger finger_msg;
            finger_msg.id = fingers[finger_id].id();
            /***name: 0:="Thumb", 1:="Index", 2:="Middle", 3:="Ring", 4:="Pinky"***/
            finger_msg.name = fingers[finger_id].type();
            finger_msg.length = fingers[finger_id].length();
            finger_msg.width = fingers[finger_id].width();
            finger_msg.velocity.x = fingers[finger_id].tipVelocity().x;
            finger_msg.velocity.y = fingers[finger_id].tipVelocity().y;
            finger_msg.velocity.z = fingers[finger_id].tipVelocity().z;
            finger_msg.pose.position.x = fingers[finger_id].tipPosition().x;
            finger_msg.pose.position.y = fingers[finger_id].tipPosition().y;
            finger_msg.pose.position.z = fingers[finger_id].tipPosition().z;

//            finger_msg.pose.position.x = fingers[finger_id].tipPosition().x - fingers[finger_id].direction().x * fingers[finger_id].tipPosition().y/ fingers[finger_id].direction().y;
//            finger_msg.pose.position.z = fingers[finger_id].tipPosition().z - fingers[finger_id].direction().z * fingers[finger_id].tipPosition().y/ fingers[finger_id].direction().y;
//            finger_msg.pose.position.y = 0;
            tf::Quaternion q_finger;
            q_finger.setRPY(fingers[finger_id].tipPosition().roll(), fingers[finger_id].tipPosition().pitch(), fingers[finger_id].tipPosition().yaw());
            finger_msg.pose.orientation.w = q_finger.getW();
            finger_msg.pose.orientation.x = q_finger.getX();
            finger_msg.pose.orientation.y = q_finger.getY();
            finger_msg.pose.orientation.z = q_finger.getZ();
            finger_msg.direction.x = fingers[finger_id].direction().x;
            finger_msg.direction.y = fingers[finger_id].direction().y;
            finger_msg.direction.z = fingers[finger_id].direction().z;
            std::cout << std::string(4, ' ') << fingerNames[fingers[finger_id].type()]<<std::endl;
            std::cout<<"Position X:"<<fingers[finger_id].tipPosition().x <<" Y:"<<fingers[finger_id].tipPosition().y <<
                       " Z: "<<fingers[finger_id].tipPosition().z << std::endl;

            //Get bones(joint position)
            for (int b = 0; b < 4; ++b) {
              Leap::Bone::Type boneType = static_cast<Leap::Bone::Type>(b);
              Leap::Bone bone = fingers[finger_id].bone(boneType);
              finger_msg.bones[b].name = b;
              finger_msg.bones[b].start.x = bone.prevJoint().x;
              finger_msg.bones[b].start.y = bone.prevJoint().y;
              finger_msg.bones[b].start.z = bone.prevJoint().z;
              finger_msg.bones[b].end.x = bone.nextJoint().x;
              finger_msg.bones[b].end.y = bone.nextJoint().y;
              finger_msg.bones[b].end.z = bone.nextJoint().z;
              finger_msg.bones[b].direction.x = bone.direction().x;
              finger_msg.bones[b].direction.y = bone.direction().y;
              finger_msg.bones[b].direction.z = bone.direction().z;

//              std::cout << std::string(6, ' ') <<  boneNames[boneType]
//                        << " bone, start: " << bone.prevJoint()
//                        << ", end: " << bone.nextJoint()
//                        << ", direction: " << bone.direction() << std::endl;
            }


            leap_msg.fingers.push_back(finger_msg);

            geometry_msgs::Point p;
            p.x = (float)fingers[finger_id].tipPosition().x;
            p.y = -(float)fingers[finger_id].tipPosition().z;
            p.z = (float)fingers[finger_id].tipPosition().y;

            marker_finger.points.push_back(p);
            trajectory_finger.points.push_back(p);
            p.x =   (float)fingers[finger_id].tipPosition().x + fingers[finger_id].direction().x * -(fingers[finger_id].tipPosition().y/fingers[finger_id].direction().y) ;
            p.y = -((float)fingers[finger_id].tipPosition().z + fingers[finger_id].direction().z * -(fingers[finger_id].tipPosition().y/fingers[finger_id].direction().y) );
            p.z = 0;
            marker_finger.points.push_back(p);
            trajectory_finger.points.push_back(p);


        }
        //marker.lifetime = ros::Duration();

        marker_pub.publish(marker_finger);
        marker_pub.publish(line_strip);
        marker_pub.publish(trajectory_finger);
    }

    //pointables
    if(!frame.pointables().isEmpty())
    {
        Leap::PointableList pointables = frame.pointables();
        for(int pointable_id = 0;pointable_id < pointables.count();pointable_id++)
        {
            std::cout << "pointable " << pointable_id << " is a ";
            std::cout << ((pointables[pointable_id].isTool())? "tool" : "finger") << std::endl;
        }
    }

    // Get gestures
    const Leap::GestureList gestures = frame.gestures();
    //CUSTOM GESTURE
    bool detected = false;
    leap_msgs::Gesture custom_gesture_msg;
    const Leap::Hand hand_a = controller.frame().hands()[0];
    Leap::Vector normal_a = hand_a.palmNormal();
    for(int i=1;i<20;i++){
        Leap::Hand hand_b = controller.frame(i).hands()[0];
        Leap::Vector normal_b = hand_b.palmNormal();
        if(normal_a.angleTo(normal_b)/3.14*180 > 80)
              detected = true;
    }
    if(detected){
        std::cout<< "CUSTOM GESTURE DETECTED"<<std::endl;
        custom_gesture_msg.gesture_type = 7;
        leap_msg.gestures.push_back(custom_gesture_msg);
        leap_gesture_pub.publish(custom_gesture_msg);
    }

    for (int g = 0; g < gestures.count(); ++g) {
        Leap::Gesture gesture = gestures[g];
        leap_msgs::Gesture gesture_msg;

        switch (gesture.type()) {
        case Leap::Gesture::TYPE_CIRCLE:
        {
            Leap::CircleGesture circle = gesture;
            std::string clockwiseness;

            if (circle.pointable().direction().angleTo(circle.normal()) <= Leap::PI/4) {
                clockwiseness = "clockwise";
            } else {
                clockwiseness = "counterclockwise";
            }

            // Calculate angle swept since last frame
            float sweptAngle = 0;
            if (circle.state() != Leap::Gesture::STATE_START) {
                Leap::CircleGesture previousUpdate = Leap::CircleGesture(controller.frame(1).gesture(circle.id()));
                sweptAngle = (circle.progress() - previousUpdate.progress()) * 2 * Leap::PI;
            }
//            std::cout << "Circle id: " << gesture.id()
//                      << ", state: " << gesture.state()
//                      << ", progress: " << circle.progress()
//                      << ", radius: " << circle.radius()
//                      << ", angle " << sweptAngle * Leap::RAD_TO_DEG
//                      <<  ", " << clockwiseness << std::endl;
            gesture_msg.gesture_type = 4;//Leap::Gesture::TYPE_CIRCLE;
            gesture_msg.id = gesture.id();
            gesture_msg.progress = circle.progress();
            gesture_msg.radius = circle.radius();
            gesture_msg.angle = sweptAngle * Leap::RAD_TO_DEG;
            gesture_msg.clockwiseness = clockwiseness;
            gesture_msg.state = circle.state();
            break;
        }
        case Leap::Gesture::TYPE_SWIPE:
        {
            Leap::SwipeGesture swipe = gesture;
//            std::cout << "Swipe id: " << gesture.id()
//                      << ", state: " << gesture.state()
//                      << ", direction: " << swipe.direction()
//                      << ", speed: " << swipe.speed() << std::endl;
            gesture_msg.gesture_type = Leap::Gesture::TYPE_SWIPE;
            gesture_msg.id = gesture.id();
            gesture_msg.direction.x = swipe.direction().x;
            gesture_msg.direction.y = swipe.direction().y;
            gesture_msg.direction.z = swipe.direction().z;
            gesture_msg.pose.position.x = swipe.position().x;
            gesture_msg.pose.position.y = swipe.position().y;
            gesture_msg.pose.position.z = swipe.position().z;
            tf::Quaternion q_swipe;
            q_swipe.setRPY(swipe.position().roll(),swipe.position().pitch(),swipe.position().yaw());
            gesture_msg.pose.orientation.w = q_swipe.getW();
            gesture_msg.pose.orientation.x = q_swipe.getX();
            gesture_msg.pose.orientation.y = q_swipe.getY();
            gesture_msg.pose.orientation.z = q_swipe.getZ();
            gesture_msg.speed = swipe.speed();
            gesture_msg.state = swipe.state();
            break;
        }
        case Leap::Gesture::TYPE_KEY_TAP:
        {
            Leap::KeyTapGesture tap = gesture;
//            std::cout << "Key Tap id: " << gesture.id()
//                      << ", state: " << gesture.state()
//                      << ", position: " << tap.position()
//                      << ", direction: " << tap.direction()<< std::endl;
            gesture_msg.gesture_type = Leap::Gesture::TYPE_KEY_TAP;
            gesture_msg.id = gesture.id();
            gesture_msg.pose.position.x = tap.position().x;
            gesture_msg.pose.position.y = tap.position().y;
            gesture_msg.pose.position.z = tap.position().z;
            tf::Quaternion q_tap;
            q_tap.setRPY(tap.position().roll(),tap.position().pitch(),tap.position().yaw());
            gesture_msg.pose.orientation.w = q_tap.getW();
            gesture_msg.pose.orientation.x = q_tap.getX();
            gesture_msg.pose.orientation.y = q_tap.getY();
            gesture_msg.pose.orientation.z = q_tap.getZ();
            gesture_msg.state = tap.state();
            break;
        }
        case Leap::Gesture::TYPE_SCREEN_TAP:
        {
            Leap::ScreenTapGesture screentap = gesture;
//            std::cout << "Screen Tap id: " << gesture.id()
//                      << ", state: " << gesture.state()
//                      << ", position: " << screentap.position()
//                      << ", direction: " << screentap.direction()<< std::endl;
            gesture_msg.gesture_type = Leap::Gesture::TYPE_SCREEN_TAP;
            gesture_msg.id = gesture.id();
            gesture_msg.pose.position.x = screentap.position().x;
            gesture_msg.pose.position.y = screentap.position().y;
            gesture_msg.pose.position.z = screentap.position().z;
            tf::Quaternion q_screentap;
            q_screentap.setRPY(screentap.position().roll(),screentap.position().pitch(),screentap.position().yaw());
            gesture_msg.pose.orientation.w = q_screentap.getW();
            gesture_msg.pose.orientation.x = q_screentap.getX();
            gesture_msg.pose.orientation.y = q_screentap.getY();
            gesture_msg.pose.orientation.z = q_screentap.getZ();
            gesture_msg.direction.x = screentap.direction().x;
            gesture_msg.direction.y = screentap.direction().y;
            gesture_msg.direction.z = screentap.direction().z;
            gesture_msg.state = screentap.state();
            break;
        }
        default:
//            std::cout << "Unknown gesture type." << std::endl;
            break;
        }
        leap_msg.gestures.push_back(gesture_msg);
        leap_gesture_pub.publish(gesture_msg);
    }



    if (!frame.hands().isEmpty() || !gestures.isEmpty()) {
        std::cout << std::endl;
    }

    mutex.lock();
    leap_msg_local = leap_msg;
    mutex.unlock();

    leap_msg_pub.publish(leap_msg);
    this->leap_data_msg = leap_msg;

    //ros::spinOnce();

}

void SampleListener::onFocusGained(const Leap::Controller& controller) {
    std::cout << "Focus Gained" << std::endl;
}

void SampleListener::onFocusLost(const Leap::Controller& controller) {
    std::cout << "Focus Lost" << std::endl;
}

void SampleListener::onDeviceChange(const Leap::Controller& controller) {
  std::cout << "Device Changed" << std::endl;
  const DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
  }
}

void SampleListener::onServiceConnect(const Leap::Controller& controller) {
  std::cout << "Service Connected" << std::endl;
}

void SampleListener::onServiceDisconnect(const Leap::Controller& controller) {
  std::cout << "Service Disconnected" << std::endl;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "leap_node_cpp");

    // Create a sample listener and controller
    SampleListener listener;
    Leap::Controller controller;

    // Have the sample listener receive events from the controller
    controller.addListener(listener);

    // Keep this process running until Enter is pressed
    //std::cout << "Press Enter to quit..." << std::endl;
    //std::cin.get();
    ros::spin();

    // Remove the sample listener when done
    controller.removeListener(listener);

    return 0;
}
