/*
Copyright (c) 2022, Marvelmind Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

// C++ STL
#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <cstdio>
#ifndef WIN32
#include <semaphore.h>
#include <fcntl.h>
#endif

extern "C"
{
    #include "marvelmind_api.h"
}

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

#include "marvelmind_ros2_msgs/srv/marvelmind_api.hpp"

class marvelmind_api_ros2 : public rclcpp::Node
{
    public:
        marvelmind_api_ros2();
        ~marvelmind_api_ros2();
    private:

        // ROS2 Setup
        void mmAPIrequestProcess(marvelmind_ros2_msgs::srv::MarvelmindAPI::Request::SharedPtr req,
            marvelmind_ros2_msgs::srv::MarvelmindAPI::Response::SharedPtr res);

        // Function prototypes
        void timer1Callback();

        void tryOpenPort(void);


        // Variables
        // Marvelmind API service
        rclcpp::Service<marvelmind_ros2_msgs::srv::MarvelmindAPI>::SharedPtr server_;

        rclcpp::TimerBase::SharedPtr m_timer1;

        std::string tty_filename;
        int64_t publish_rate_in_hz;

        uint32_t mm_api_version;

        bool openPortTryNeeded;
};
