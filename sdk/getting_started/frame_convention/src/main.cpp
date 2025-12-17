// Copyright (C) 2025 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * Simple frame convention example for the CreOS client library.
 *
 * This example demonstrates how to connect to the CreOS server and subscribe
 * to the odometry messages and convert the odometry from ENU/FRD to NED/FRD
 * frame convention.
 ****************************************************************************/
#include <creos/client.hpp>
#include <creos/robot_clock.hpp>
#include <creos/frame_convention.hpp>
#include <creos/messages/odometry.hpp>
#include <iostream>

class FrameConventionClient
{
public:
    FrameConventionClient(int argc, char **argv)
        : _client(CreateClient(GetHost(argc, argv))), _prev_time(creos::RobotClock::now())
    {
        // Register a callback function for the odometry messages.
        _client.sensors()->subscribeToOdometry(
            std::bind(&FrameConventionClient::CallbackOdometry, this, std::placeholders::_1));
    }

    static creos::Client CreateClient(const std::string_view host)
    {
        // Connect to the CreOS server.
        if(host.empty())
        {
            return creos::Client();
        }
        return creos::Client(host);
    }

    static std::string_view GetHost(int argc, char **argv)
    {
        // Get the hostname from the environment variable or the command line argument.
        std::string_view host = "";

        if(argc >= 2)
        {
            host = argv[1];
        }
        return host;
    }

    void CallbackOdometry(const creos_messages::Odometry &message)
    {
        // Only process new messages once a second.
        if(message.timestamp - _prev_time < std::chrono::seconds(1))
        {
            return;
        }

        _prev_time = message.timestamp;

        // Print the received odometry message.
        std::cout << "Odometry received [ENU/FLU]: \n"
                  << " - Position: [" << message.pose.position.x << ", " << message.pose.position.y
                  << ", " << message.pose.position.z << "] \n"
                  << " - Orientation (quaternion): [" << message.pose.orientation.w << ", "
                  << message.pose.orientation.x << ", " << message.pose.orientation.y << ", "
                  << message.pose.orientation.z << "] \n"
                  << " - Linear Velocity: [" << message.twist.linear.x << ", "
                  << message.twist.linear.y << ", " << message.twist.linear.z << "], \n"
                  << " - Angular Velocity: [" << message.twist.angular.x << ", "
                  << message.twist.angular.y << ", " << message.twist.angular.z << "]" << std::endl;

        // Convert the received odometry from ENU/FLU to NED/FRD frame convention.
        creos_messages::Odometry odom_ned_frd = creos::ConvertEnuFluToNedFrd(message);

        // Print the converted odometry message.
        std::cout << "Odometry converted [NED/FRD]: \n"
                  << " - Position: [" << odom_ned_frd.pose.position.x << ", "
                  << odom_ned_frd.pose.position.y << ", " << odom_ned_frd.pose.position.z << "] \n"
                  << " - Orientation (quaternion): [" << odom_ned_frd.pose.orientation.w << ", "
                  << odom_ned_frd.pose.orientation.x << ", " << odom_ned_frd.pose.orientation.y
                  << ", " << odom_ned_frd.pose.orientation.z << "] \n"
                  << " - Linear Velocity: [" << odom_ned_frd.twist.linear.x << ", "
                  << odom_ned_frd.twist.linear.y << ", " << odom_ned_frd.twist.linear.z << "], \n"
                  << " - Angular Velocity: [" << odom_ned_frd.twist.angular.x << ", "
                  << odom_ned_frd.twist.angular.y << ", " << odom_ned_frd.twist.angular.z << "]"
                  << std::endl;
    }

private:
    creos::Client                 _client;
    creos::RobotClock::time_point _prev_time;
};

int main(int argc, char **argv)
{
    // Create the FrameConventionClient which connects to the CreOS server and subscribes to
    // odometry messages.
    FrameConventionClient frame_convention_client(argc, argv);

    while(true)
    {
        sleep(1);
    }
    return 0;
}
