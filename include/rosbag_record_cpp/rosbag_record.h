/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Allows ROS applications to programmatically use rosbag as a thin wrapper without dealing with topic subscriptions
*/

#ifndef ROSBAG_RECORD_CPP__ROSBAG_RECORD_
#define ROSBAG_RECORD_CPP__ROSBAG_RECORD_

// ROS
#include <ros/ros.h>
#include <rosbag_record_cpp/recorder.h>
#include <rosbag/exceptions.h>
#include <rosbag/recorder.h>

// Boost
#include <boost/program_options.hpp>
#include <boost/thread.hpp>

// C++
#include <string>
#include <sstream>

namespace po = boost::program_options;

namespace rosbag_record_cpp
{

class ROSBagRecord
{
private:

  // Node Handles
  ros::NodeHandle nh_; // no namespace

  // Flag for stopping the threads
  bool stop_recording_;

  boost::thread worker_thread_;

public:

  /**
   * \brief Constructor
   */
  ROSBagRecord();
  ~ROSBagRecord();

  void startRecording(rosbag_record_cpp::RecorderOptions opts);

  void thread(rosbag_record_cpp::RecorderOptions opts);

  void stopRecording();

};

} // namespace

#endif
