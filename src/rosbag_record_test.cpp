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
   Desc:   Testing main executable for rosbag_record functionality
*/

#include <rosbag_record_cpp/rosbag_record.h>

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("rosbag_record_test","rosbag_record_test");

  ros::init(argc, argv, "rosbag_record_test");

  // Allow the action server to recieve and send ros messages
  //ros::AsyncSpinner spinner(4);
  //spinner.start();

  rosbag_record_cpp::ROSBagRecord tester;

  // Set the options for this recording
  rosbag::RecorderOptions opts;
  opts.topics.push_back("/robot/left_velocity_trajectory_controller/state");
  opts.append_date = false;

  int counter = 0;
  while(ros::ok() && counter < 5)
  {
    ROS_INFO_STREAM_NAMED("rosbag_record_test","Recording test " << counter);

    opts.prefix = "file" + boost::to_string(counter); // name of file to output to

    // Start
    tester.startRecording(opts);

    ros::Duration(2.0).sleep();

    // Stop
    tester.stopRecording();
    counter ++;
  }

  ROS_INFO_STREAM_NAMED("rosbag_record_test","Shutting down.");

  return 0;
}

