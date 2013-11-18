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

#include <rosbag_record_cpp/rosbag_record.h>

namespace rosbag_record_cpp
{

ROSBagRecord::ROSBagRecord()
 : stop_recording_(false)
{

}

ROSBagRecord::~ROSBagRecord()
{
}

// Start the data collection
void ROSBagRecord::startRecording(rosbag::RecorderOptions opts)
{
  stop_recording_ = false;

  ROS_INFO_STREAM_NAMED("rosbag_record","Starting rosbag record");
  worker_thread_ = boost::thread(&ROSBagRecord::thread, this, opts);
}

void ROSBagRecord::thread(rosbag::RecorderOptions opts)
{
  // Run the recorder
  rosbag::Recorder recorder(opts);
  recorder.run(&stop_recording_);
}

void ROSBagRecord::stopRecording()
{
  ROS_INFO_STREAM_NAMED("rosbag_record","Stopping rosbag record");
  stop_recording_ = true;
  worker_thread_.join();
}

} // namespace

