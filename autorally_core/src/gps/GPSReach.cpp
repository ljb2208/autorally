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
 * @file GPSHemisphere.cpp
 * @author Brian Goldfain <bgoldfain3@gatech.edu>
 * @date November 5, 2013
 * @copyright 2013 Georgia Institute of Technology
 * @brief Implementation of GPSHemisphere class
 *
 ***********************************************/
#include "GPSReach.h"

#include <time.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

/**
 * This NMEA GPS Interface program currently for the Reach RTK GNSS
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpsBase");
  ros::NodeHandle nh;

  GPSReach gpsInterface(nh);

  ros::spin();

  return 0;
}

GPSReach::GPSReach(ros::NodeHandle &nh):
  m_previousCovTime(ros::Time::now()),
  m_mostRecentRTK(ros::Time::now()),
  m_rtkEnabled(true)
{
  std::string nodeName = ros::this_node::getName();
  std::string mode;
  std::string portPathA = "";
  std::string portPathB = "";

  _step = 0;

  //default values reasonable for a crappy gps
  nh.param<double>(nodeName+"/accuracyRTK", m_accuracyRTK, 0.02);
  nh.param<double>(nodeName+"/accuracyWAAS", m_accuracyWAAS, 0.6);
  nh.param<double>(nodeName+"/accuracyAutonomous", m_accuracyAutonomous, 2.5);
  nh.param<double>(nodeName+"/gpsTimeOffset", m_gpsTimeOffset, 0.0);
  nh.param<std::string>(nodeName+"/utcSource", m_utcSource, "GPZDA");
  nh.param<bool>(nodeName+"/showGsv", m_showGsv, "false");


  if(!nh.getParam(nodeName+"/mode", mode) ||
     !nh.getParam(nodeName+"/primaryPort/portPath", portPathA))
  {
    ROS_ERROR("GPSReach: could not find mode or portPaths");
  }
  {
    if(mode == "base")
    {
      m_navSatFix.header.frame_id = "gpsBase";
      m_bIsBase = true;

      m_rtkCorrection.layout.data_offset = 0;
      m_rtkCorrection.layout.dim.push_back(std_msgs::MultiArrayDimension());
      m_timeUTC.source = "gps";

      nh.getParam(nodeName+"/correctionPort/portPath", portPathB);

      /* Have to init serial ports after publishers are connected otherwise
       * if a message is received from serial before the associated publisher
       * is connected, an exception occurs and the node crashes.
       */
      m_portA.init(nh, nodeName, "primaryPort", "Reach", portPathA, true);
      m_portB.init(nh, nodeName, "correctionPort", "Reach", portPathB, true);
      
      m_rtcm3Pub = nh.advertise<std_msgs::ByteMultiArray>("gpsBaseRTCM3", 5);
      m_statusPub = nh.advertise<sensor_msgs::NavSatFix>("gpsBaseStatus", 5);
      m_utcPub = nh.advertise<sensor_msgs::TimeReference>("utc", 5);

      m_rtkStatusTimer = nh.createTimer(ros::Duration(1.0),
                                        &GPSReach::rtkStatusCallback,
                                        this);
      m_portA.registerDataCallback(
                      boost::bind(&GPSReach::gpsInfoCallback, this));
      m_portB.registerDataCallback(
                      boost::bind(&GPSReach::publishRTCMData, this));
      
      //  m_refLocTimer = nh.createTimer(ros::Duration(60.0),
//                    &GPSHemisphere::updateReferenceLocationCallback,
//                    this);
    } else if(mode == "rover")
    {
      /*rover gets base location through RTK corrections, updating ref location
       * is not needed
       */
      m_bIsBase = false;
      m_navSatFix.header.frame_id = "gpsRover";

      /* Have to init serial ports after publishers are connected otherwise
       * if a message is received from serial before the associated publisher
       * is connected, an exception occurs and the node crashes.
       */
      m_portA.init(nh, nodeName, "primaryPort", "ReachRover", portPathA, true);          
      
      m_statusPub = nh.advertise<sensor_msgs::NavSatFix>("gpsRoverStatus", 5);

      m_rtcm3Sub = nh.subscribe("gpsBaseRTCM3", 5,
                              &GPSReach::rtcmCorrectionCallback,
                              this);
      m_portA.registerDataCallback(
                      boost::bind(&GPSReach::gpsInfoCallback, this));
    } else
    {
      ROS_ERROR("GPSReach: unrecognized mode of operation:%s", mode.c_str());
    }
  }

  if(!m_portA.connected())
  {
    ROS_ERROR("GPSReach: primary serial port is not open");
  }

  if(!m_portB.connected() && m_bIsBase)
  {
    ROS_ERROR("GPSReach: secondary serial port is not open");
  }

  m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
  m_navSatFix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  m_navSatFix.latitude = 0.0;
  m_navSatFix.longitude = 0.0;
  m_navSatFix.altitude = 0.0;
  m_navSatFix.position_covariance_type =
              sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  //set real high covariances for now
  m_navSatFix.position_covariance[0] = 99999;
  m_navSatFix.position_covariance[4] = 99999;
  m_navSatFix.position_covariance[8] = 99999;


  time_t t = time(NULL);
  tm* timePtr = gmtime(&t);

  //since this is only done once, the published utc time messages will not
  //be correct if the code is run overnight receiving GPGSA or GPGGA for timing
  m_secondsToToday = 360*24*(timePtr->tm_year);
}

GPSReach::~GPSReach()
{

}


void GPSReach::gpsInfoCallback()
{
  m_portA.lock();  

  //make sure data is framed (we expect only NMEA 0183 messages here)
  if(!m_portA.m_data.empty())
  {
    size_t len = m_portA.m_data.length();

    ROS_DEBUG_STREAM("GPSReach:: data length " << len);    

    for (uint i=0; i < len; i++)
    {
      uint8_t data = m_portA.m_data[i];      

      if (data == PREAMBLE1 && _step == 0)
      {
        _step++;
        continue;
      }
      else if (data == PREAMBLE2 && _step == 1)
      {
        _step++;
        continue;
      }
      else if (_step == 2)
      {
        _msg_id = data;
        _ck_b = _ck_a = data;
        _step++;
        continue;
      }
      else if (_step == 3)
      {
        _step++;
        _ck_b += (_ck_a += data);                   // checksum byte
        _payload_length = data; 
        continue;
      }
      else if (_step == 4)
      {
        _step++;
        _ck_b += (_ck_a += data);                   // checksum byte
        _payload_length += (uint16_t)(data<<8);
        _payload_counter = 0; // prepare to receive payload
        continue;
      }
      else if (_step == 5)
      {
        _ck_b += (_ck_a += data);                   // checksum byte
        if (_payload_counter < sizeof(_buffer)) {
            _buffer[_payload_counter] = m_portA.m_data[i];
        }
        if (++_payload_counter == _payload_length)
          _step++;
        
        continue;
      }
      else if (_step == 6)
      {
        _step++;
        if (_ck_a != data)
        {
          ROS_WARN_STREAM("GPSReach:: Invalid Checksum");
        }
        continue;
      }
      else if (_step == 7)
      {
        _step = 0;        

        if (_ck_b != data)
        {
          ROS_WARN_STREAM("GPSReach:: Invalid Checksum");
          continue;
        }        

        m_portA.m_data.erase(0, i + 1);
        i = 0;
        len = m_portA.m_data.length();

        // ROS_DEBUG_STREAM("GPSReach: i " << i << " len: " << len << "data: " << m_portA.m_data.c_str());
        
        processGPSMessage(_msg_id);
        ROS_DEBUG_STREAM("GPSReach:: Successfully parsed message of type: " << (int) _msg_id << " remaining data: " << len); 
        continue;
      }
    }
  }
  m_portA.unlock();
}

void GPSReach::processGPSTime(uint32_t millisecs, uint16_t week)
{
  m_gpsTime = SECONDS_TO_GPS_EPOCH;
  m_gpsTime += week * SECONDS_IN_WEEK
  m_gpsTime += millisecs / 1000;

  if (m_bIsBase)
  {
      m_timeUTC.header.stamp = ros::Time::now();
      m_timeUTC.time_ref = ros::Time(m_gpsTime);
      m_utcPub.publish(m_timeUTC);
  }

  // ROS_DEBUG_STREAM("GPS Time Raw: " << millisecs << " week: " << week);
  // ROS_DEBUG_STREAM("GPS Time " << asctime(gmtime(&m_gpsTime)));
}

void GPSReach::processGPSMessage(int msgId)
{
  if (msgId == MSG_POS)
  {
      m_navSatFix.latitude = _buffer.pos.latitude;
      m_navSatFix.longitude = _buffer.pos.longitude;
      m_navSatFix.altitude = _buffer.pos.altitude_msl;

      m_navSatFix.header.stamp == ros::Time::now();
      m_statusPub.publish(m_navSatFix);

      //m_navSatFix.header.stamp = ros::Time(((double)((int)(ros::Time::now().toSec() + m_gpsTimeOffset) / 86400) * 86400) + messageTime + m_gpsTimeOffset);
  }
  else if (msgId == MSG_DOPS)
  {      
      std::string hdop = std::to_string(_buffer.dops.hDOP);
      std::string vdop = std::to_string(_buffer.dops.vDOP);
      std::string pdop = std::to_string(_buffer.dops.pDOP);
      m_portA.diag("DOPS: HDOP:", hdop);        
      m_portA.diag("DOPS: VDOP:", vdop);        
      m_portA.diag("DOPS: PDOP:", pdop);              
  }
  else if (msgId == MSG_STAT)
  {   
    std::string fixType = "No Fix"; 
    switch(_buffer.stat.fix_type)
    {
      case 0: // no fix
        m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        break;
      case 1: // Single        
        fixType = "Single"; 
        m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        break;
      case 2: // Float   
        fixType = "Float";      
        m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        break;
      case 3: // RTK Fix
        fixType = "RTK Fix";      
        m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        break;
      default:
        m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        break;
    }

    processGPSTime(_buffer.stat.time, _buffer.stat.week);
    m_navSatFix.header.stamp == ros::Time(m_gpsTime);

    m_statusPub.publish(m_navSatFix);   

    std::string tm = boost::lexical_cast<std::string>(m_gpsTime);
    std::string numSats = std::to_string(_buffer.stat.satellites);
    m_portA.diag("Status: UTC HHMMSS.SS:", tm);
    m_portA.diag("Status: GPS mode indicator:", fixType);        
    m_portA.diag("Status: # of satellites:", numSats);   
  }
  else if (msgId == MSG_RTK)
  {    
    
  }
}

void GPSReach::publishRTCMData()
{
  /*RTCM 3.0 frame structure:
              bits 0 - 7 - header
              bits 8 - 13 - reserved
              bits 14 - 23 - message length
              bits 24 - 23+length - message
              bits 24+length - 24+length+23 - 24 bit bit CRC
              first 6 bits of message body are message type,
              total frame size (bytes): message length + 6
  */    

  m_portB.lock();

  if(m_portB.m_data.size() > 6)
  {
      std::string s;
      s.push_back(0xd3);
      s.push_back(0x00);
      size_t start = m_portB.m_data.find(s);
      unsigned int msgLen = (unsigned int)(((m_portB.m_data[start + 2])&0xff));      
      size_t end = start + msgLen + 6;

      while (start != std::string::npos && end != std::string::npos)
      {
          unsigned int type = (unsigned int)(((m_portB.m_data[start + 3])&0xff)<<4) +
                          (unsigned int)(((m_portB.m_data[start + 4])&0xf0)>>4);          
                   
          m_portB.tick("RTCM3.0 type "+boost::lexical_cast<std::string>(type));
         
          //fill in structure to send message
          m_rtkCorrection.layout.dim.front().label = "RTCM3.0 " +
                          boost::lexical_cast<std::string>(type);
      
          m_rtkCorrection.layout.dim.front().size = msgLen;
          m_rtkCorrection.layout.dim.front().stride = 1*(msgLen);
          m_rtkCorrection.data.resize(msgLen);
          memcpy(&m_rtkCorrection.data[0], &m_portB.m_data[start], msgLen);

          m_rtcm3Pub.publish(m_rtkCorrection);          
          ROS_DEBUG_STREAM("RTCM Data published. Len: " << (int) msgLen);
          ROS_DEBUG_STREAM("Len pre erase: " << (int) m_portB.m_data.size());
          m_portB.m_data.erase(0, end);

          ROS_DEBUG_STREAM("Len post erase: " << (int) m_portB.m_data.size());

          // check to see if enough data is still on the port
          if (m_portB.m_data.size() <= 6)
          {
            break;
          }

          start = m_portB.m_data.find(s);
          msgLen = (unsigned int)(((m_portB.m_data[start + 2])&0xff));
          msgLen += 6; // add in header for total message len;

          end = start + msgLen + 6;          
      }
  }


  m_portB.unlock();                  
}

void GPSReach::rtcmCorrectionCallback(const std_msgs::ByteMultiArray& msg)
{
  ROS_WARN("write RTCM3 correction data");
  m_portA.lock();
  m_portA.tick("Incoming Correction Data");
  m_mostRecentRTK = ros::Time::now();
  m_portA.writePort( reinterpret_cast<const unsigned char*>(&msg.data[0]),
                     msg.layout.dim[0].size);
  m_portA.unlock();
}
void GPSReach::rtkStatusCallback(const ros::TimerEvent& /*time*/)
{
  // //query the current RTK transmission status
  // unsigned char cmd[10] = "$JRTK,6\r\n";
  // m_portA.writePort(cmd,10);

  //if there are no recent RTK corrections, switch to satellite augmentation
//  if((ros::Time::now()-m_mostRecentRTK).toSec() > 120 && m_rtkEnabled)
//  {
//    unsigned char mode[14] = "$JDIFF,WAAS\r\n";
//    m_portA.writePort(cmd,14);
//    m_rtkEnabled = false;
//    m_portA.diag_ok("Switching to WAAS corrections");
//  } else if((ros::Time::now()-m_mostRecentRTK).toSec() < 5 && !m_rtkEnabled)
//  {
//    unsigned char mode[16] = "$JDIFF,BEACON\r\n";
//    m_portA.writePort(cmd,16);
//    m_rtkEnabled = true;
//    m_portA.diag_ok("Switching to RTK corrections");
//  }
}

void GPSReach::updateReferenceLocationCallback(const ros::TimerEvent& /*time*/)
{
  //if the unit has augmented position data, update the current reference
  //location
  if(m_navSatFix.status.status > sensor_msgs::NavSatStatus::STATUS_FIX)
  {
    // unsigned char cmd[12] = "$JRTK,1,P\r\n";
    // m_portA.writePort(cmd, 12);
  }
  return;
}