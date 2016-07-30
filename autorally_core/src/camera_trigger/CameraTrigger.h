/*
* Software License Agreement (BSD License)
* Copyright (c) 2016, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file CameraTrigger.h
 * @author Kamil Saigol <kamilsaigol@gatech.edu>
 * @date July 24, 2016
 * @copyright 2016 Georgia Institute of Technology
 * @brief
 *
 * @details This file contains the CameraTrigger class.
 ***********************************************/

#ifndef CAMERA_TRIGGER
#define CAMERA_TRIGGER

#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <autorally_core/SerialInterfaceThreaded.h>

namespace autorally_core
{

class CameraTrigger: public nodelet::Nodelet
{
public:
    virtual void onInit();

private:
    ros::NodeHandle m_nodeHandlePvt;
    SerialInterfaceThreaded m_port; ///< Serial port interface to camera trigger controller (Micro)
    int m_triggerFPS;               ///< Camera trigger frequency
    ros::Time m_lastTime;           ///< Keep track of time FPS was accessed

    void triggerCallback();
}

}

#endif // CAMERA_TRIGGER
