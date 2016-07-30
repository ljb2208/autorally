/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
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
 * @file CameraTrigger.cpp
 * @author Kamil Saigol <kamilsaigol@gatech.edu>
 * @date July 24, 2016
 * @copyright 2016 Georgia Institute of Technology
 * @brief
 *
 * @details Contains CameraTrigger class implementation.
 ***********************************************/

#include "CameraTrigger.h"
#include <boost/bind.hpp>

PLUGINLIB_DECLARE_CLASS(autorally_core, CameraTrigger, autorally_core::CameraTrigger, nodelet::Nodelet)

void autorally_core::CameraTrigger::onInit()
{
    ros::NodeHandle node_handle = getNodeHandle();
    m_nodeHandlePvt = getPrivateNodeHandle();
    std::string port;
    if(!node_handle.getParam(getName() + "/port", port) ||
       !node_handle.getParam(getname() + "triggerFPS", m_triggerFPS))
    {
        NODELET_ERROR("CameraTrigger: could not get camera trigger interface port or trigger FPS");
    }

    m_port.init(node_handle, getName(), "", "CameraTrigger", port, true);
    m_port.registerDataCallback(boost::bind(&CameraTrigger::triggerCallback, this));
}

void autorally_core::CameraTrigger::triggerCallback()
{
    if (m_lastTime + ros::Duration(2.0) > ros::Time::now())
    {
        m_lastTime = ros::Time::now();

        // set FPS
        int newFPS = 0;
        m_nodeHandlePvt.getParam("triggerFPS", newFPS);
        if(newFPS != m_triggerFPS)
        {
            m_triggerFPS = newFPS;
            m_port.lock();
            m_port.writePort(std::to_string(m_triggerFPS) + "\r\n");
            m_port.unlock();
        }
    }

    m_port.diag("Triggering FPS", std::to_string(m_triggerFPS));
}
