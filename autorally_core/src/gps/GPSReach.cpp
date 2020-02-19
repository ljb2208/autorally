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

void printMessage(const char* message, int size)
{
  for (int i = 0; i < size; i++)
  {
    	printf("%x ", message[i]&0xFF );
  }
  printf("\n");
}
void printMessage(const std::string &message)
{
  printMessage(message.c_str(), message.size());
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
     !nh.getParam(nodeName+"/primaryPort/portPath", portPathA) )
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
      m_portA.init(nh, nodeName, "primaryPort", "Reach", portPathA, true);          
      
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

  if(!m_portB.connected())
  {
    ROS_ERROR("GPSReach: primary serial port is not open");
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

    ROS_WARN_STREAM("GPSReach:: data length " << len);

    for (uint i=0; i < len; i++)
    {
      uint8_t data = m_portA.m_data[i];

      //ROS_WARN_STREAM("GPSReach:: step " << _step << " data " << (char) data);


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

        m_portA.m_data.erase(0, i);
        i = 0;
        len = m_portA.m_data.length();

        if (_ck_b != data)
        {
          ROS_WARN_STREAM("GPSReach:: Invalid Checksum");
          continue;
        }

        processGPSMessage(_msg_id);
        ROS_WARN_STREAM("GPSReach:: Successfully parsed message of type: " << (int) _msg_id); 
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

  ROS_WARN_STREAM("GPS Time Raw: " << millisecs << " week: " << week);
  ROS_WARN_STREAM("GPS Time " << asctime(gmtime(&m_gpsTime)));
}

void GPSReach::processGPSMessage(int msgId)
{
  switch(msgId)
  {
    case MSG_VER:
      //erb_ver ver = (erb_ver) buffer;
      ROS_WARN_STREAM("ERB Version: Time: " << _buffer.ver.time << " Ver: " <<  (int) _buffer.ver.ver_high << ":" << (int) _buffer.ver.ver_medium << ":" << (int)_buffer.ver.ver_low);
      break;
    case MSG_DOPS:
      ROS_WARN_STREAM("DOPS: " << (int) _buffer.dops.hDOP);
      break;
    case MSG_STAT:
      ROS_WARN_STREAM("GPS Status: Fix Type: " << (int) _buffer.stat.fix_type << "Fix Status: " << (int) _buffer.stat.fix_status << " Sats: " << (int) _buffer.stat.satellites << " Time: " << _buffer.stat.time << " Week: " << _buffer.stat.week);
      break;
    case MSG_POS:
      ROS_WARN_STREAM("GPS Pos: Long: " << _buffer.pos.longitude << " Lat: " << _buffer.pos.latitude << " Alt: " << _buffer.pos.altitude_msl);
      break;

  }

  if (msgId == MSG_POS)
  {
      m_navSatFix.latitude = _buffer.pos.latitude;
      m_navSatFix.longitude = _buffer.pos.longitude;
      m_navSatFix.altitude = _buffer.pos.altitude_msl;

      //m_navSatFix.header.stamp = ros::Time(((double)((int)(ros::Time::now().toSec() + m_gpsTimeOffset) / 86400) * 86400) + messageTime + m_gpsTimeOffset);
  }
  else if (msgId == MSG_DOPS)
  {

  }
  else if (msgId == MSG_STAT)
  {
    //m_portA.diag("GPS Status: Fix Type", _Buffer.stat.fix_type);
    switch(_buffer.stat.fix_type)
    {
      case 0: // no fix
        m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        break;
      case 1: // Single        
      case 2: // Float        
      case 3: // RTK Fix
        m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        break;
      default:
        m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        break;
    }

    processGPSTime(_buffer.stat.time, _buffer.stat.week);
    m_navSatFix.header.stamp == ros::Time(m_gpsTime);

    m_statusPub.publish(m_navSatFix);
  }
  else if (msgId == MSG_RTK)
  {
    // ROS_WARN("Process RTK");
    // publishRTCMData();
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
      size_t end = m_portB.m_data.find(s, start + 1);

      while (start != std::string::npos && end != std::string::npos)
      {
          unsigned int type = (unsigned int)(((m_portB.m_data[start + 3])&0xff)<<4) +
                          (unsigned int)(((m_portB.m_data[start + 4])&0xf0)>>4);
          int len = end - start;
          m_portB.tick("RTCM3.0 type "+boost::lexical_cast<std::string>(type));
         
          //fill in structure to send message
          m_rtkCorrection.layout.dim.front().label = "RTCM3.0 " +
                          boost::lexical_cast<std::string>(type);
      
          m_rtkCorrection.layout.dim.front().size = len;
          m_rtkCorrection.layout.dim.front().stride = 1*(len);
          m_rtkCorrection.data.resize(len);
          memcpy(&m_rtkCorrection.data[0], &m_portB.m_data[start], len);

          m_rtcm3Pub.publish(m_rtkCorrection);
          ROS_DEBUG_STREAM("RTCM Data published. Len: " << (int) len);
          m_portB.m_data.erase(0, end);

          start = m_portA.m_data.find(s);
          end = m_portA.m_data.find(s, start + 1);
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

/* void GPSReach::processGPSMessage(std::string& msg)
{
  if(msg.length() == 0)
  {
    ROS_WARN("GPSReach: recieved empty message.");
    return;
  }
  std::vector<std::string> tokens;
  boost::split(tokens, msg, boost::is_any_of(",*"));
  std::string msgType = tokens[0];
  if(msgType == "GPGGA")
  {
    if(tokens.size() < 15)
    {
      ROS_WARN("GPSReach: %s wrong token count %lu", msgType.c_str(), tokens.size());
      return;
    }
    m_portA.tick(msgType);
    if(msgType != m_statusPositionSource)
    {
      ROS_WARN("GPSReach: using %s for fix data, ignoring %s",
               m_statusPositionSource.c_str(),
               msgType.c_str()); 
      return;
    }
    if( atoi(tokens[1].c_str()) == 0)
    {
      m_navSatFix.latitude = 0.0;
      m_navSatFix.longitude = 0.0;
      m_navSatFix.altitude = 0.0;
      m_navSatFix.header.stamp = ros::Time::now();
      m_portA.diag(msgType + " UTC HHMMSS.SS:", "-");
      m_portA.diag(msgType + " quality:", processQuality("0"));
      m_portA.diag(msgType + " # of satellites:", "0");
      m_portA.diag(msgType + " HDOP:", "-");
      m_portA.diag(msgType + " diff correction age (s):", "-");
      m_portA.diag(msgType + " diff ref station ID:", "-");
    } else
    {
      processUTC(tokens[1], msgType);
      m_portA.diag(msgType + " UTC HHMMSS.SS:", tokens[1].c_str());
      m_navSatFix.latitude = processLatitude(tokens[2],tokens[3]);
      m_navSatFix.longitude = processLongitude(tokens[4],tokens[5]);
      m_portA.diag(msgType + " quality:", processQuality(tokens[6]));
      m_portA.diag(msgType + " # of satellites:", tokens[7].c_str());
      m_portA.diag(msgType + " HDOP:", tokens[8].c_str());
      
      m_navSatFix.altitude = processAltitude(tokens[9], tokens[10], tokens[11], tokens[12]);
      if(fabs(m_navSatFix.altitude) < 0.001 || fabs(m_navSatFix.latitude) < 0.001 || fabs(m_navSatFix.longitude) < 0.001)
      {
        return;
      }
      
      //quality token
      if(tokens[6] != "0" && tokens[6] != "1")
      {
        if(tokens.size() < 15)
        {
          ROS_WARN("GPSReach: wrong token count 3 in: %s", msg.c_str());
          return;
        }
        m_portA.diag(msgType + " diff correction age (s):", tokens[13].c_str());
        m_portA.diag(msgType + " diff ref station ID:", tokens[14].c_str());
      } else
      {
        m_portA.diag(msgType + " diff correction age (s):", "-");
        m_portA.diag(msgType +  " diff ref station ID:", "-");
      }
      double messageTime = GetUTC(tokens[1]);
      m_navSatFix.header.stamp = ros::Time(((double)((int)(ros::Time::now().toSec() + m_gpsTimeOffset) / 86400) * 86400) + messageTime + m_gpsTimeOffset);
      //std::cout << (double)((int)(ros::Time::now().toSec() + m_gpsTimeOffset) / 86400) << "Week" << messageTime << time << std::endl;
      //std::cout << (ros::Time::now().toSec()) << std::endl;
    }
    double messageAge = m_navSatFix.header.stamp.toSec() - ros::Time::now().toSec();
    // Abandon our timestamp if its too far off
    if (messageAge > 1.0 || messageAge < -1.0){
      m_navSatFix.header.stamp = ros::Time::now();
      ROS_ERROR("GPS message too old! %f seconds", messageAge);
    }
    
    try
    {
      m_portA.diag("GPS Message Age (s)", (boost::lexical_cast<std::string>(messageAge)).c_str());
    } catch(const boost::bad_lexical_cast &)
    {
      ROS_ERROR_STREAM("GPSReach failed GPS message age lexical cast");
      m_portA.diag_warn("GPSReach failed GPS message age lexical cast");
      return;
    }
    m_statusPub.publish(m_navSatFix);
    m_portA.tick("Publishing navSatFix");
  } else if(msgType == "GPGNS")
  {
    if(tokens.size() < 15)
    {
      ROS_WARN("GPSReach: %s wrong token count %lu", msgType.c_str(), tokens.size());
      return;
    }
    m_portA.tick(msgType);
    if(msgType != m_statusPositionSource)
    {
      ROS_WARN("GPSReach: using %s for fix data, ignoring %s",
               m_statusPositionSource.c_str(),
               msgType.c_str()); 
      return;
    }
    if( atoi(tokens[1].c_str()) == 0)
    {
      m_navSatFix.latitude = 0.0;
      m_navSatFix.longitude = 0.0;
      m_navSatFix.altitude = 0.0;
      m_navSatFix.header.stamp = ros::Time::now();
      m_portA.diag(msgType + " UTC HHMMSS.SS:", "-");
      m_portA.diag(msgType + " GPS mode indicator:", "no fix");
      m_portA.diag(msgType + " GLONASS mode indicator:", "no fix");
      m_portA.diag(msgType + " # of satellites:", "0");
      m_portA.diag(msgType + " HDOP:", "-");
      m_portA.diag(msgType + " diff correction age (s):", "-");
      m_portA.diag(msgType + " diff ref station ID:", "-");
      m_portA.ERROR();
      //navigational status should be unsafe when no fix
      if(tokens[13] == "U")
      {
        m_portA.diag(msgType + " Navigational status:", "U - unsafe");
      } else
      {
        m_portA.diag(msgType + "Unknown no fix navigational status:", tokens[13]);
      }
    } else
    {
      processUTC(tokens[1], msgType);
      m_portA.diag(msgType + " UTC HHMMSS.SS:", tokens[1].c_str());
      m_navSatFix.latitude = processLatitude(tokens[2],tokens[3]);
      m_navSatFix.longitude = processLongitude(tokens[4],tokens[5]);
      
      if(tokens[6].size() >= 1)
      {
        m_portA.diag(msgType + " GPS mode indicator:", processMode(tokens[6].substr(0,1)));
      }
      if(tokens[6].size() == 2)
      {
        m_portA.diag(msgType + " GLONASS mode indicator:", processMode(tokens[6].substr(1,1)));
      }
      m_portA.diag(msgType + " # of satellites:", tokens[7].c_str());
      m_portA.diag(msgType + " HDOP:", tokens[8].c_str());
      
      try
      {
        m_navSatFix.altitude = boost::lexical_cast<double>(tokens[9]) +
                               boost::lexical_cast<double>(tokens[10]);
      } catch(const boost::bad_lexical_cast &)
      {
        m_portA.diag_error("GPSReach::GPGNS bad altitude lexical cast");
        ROS_ERROR("GPSReach::GPGNS bad altitude lexical cast");
        return;
      } 
      
      if((tokens[6][0] == 'D' ||
          tokens[6][0] == 'P' ||
          tokens[6][0] == 'R' ||
          tokens[6][0] == 'F' || 
          tokens[6][1] == 'D' ||
          tokens[6][1] == 'P' ||
          tokens[6][1] == 'R' ||
          tokens[6][1] == 'F') )
      {
        m_portA.diag(msgType + " diff correction age (s):", tokens[11].c_str());
        m_portA.diag(msgType + " diff ref station ID:", tokens[12].c_str());
      }else
      {
        m_portA.diag(msgType + " diff correction age (s):", "-");
        m_portA.diag(msgType +  " diff ref station ID:", "-");
      }
      if(tokens[13] == "S")
      {
        m_portA.diag(msgType + " Navigational status:", "S - safe");
      } else if(tokens[13] == "C")
      {
        m_portA.diag(msgType + " Navigational status:", "C - caution");
      } else if(tokens[13] == "U")
      {
        m_portA.diag(msgType + " Navigational status:", "U - unsafe");
      } else if(tokens[13] == "V")
      {
        m_portA.diag(msgType + " Navigational status:", "V - not valid");
      } else
      {
        m_portA.diag(msgType + "Unknown Navigational status:", tokens[13]);
      }
      double messageTime = GetUTC(tokens[1]);
      m_navSatFix.header.stamp = ros::Time(((double)((int)(ros::Time::now().toSec() + m_gpsTimeOffset) / 86400) * 86400) + messageTime + m_gpsTimeOffset);
      //std::cout << (double)((int)(ros::Time::now().toSec() + m_gpsTimeOffset) / 86400) << "Week" << messageTime << time << std::endl;
      //std::cout << (ros::Time::now().toSec()) << std::endl;
    }
    double messageAge = m_navSatFix.header.stamp.toSec() - ros::Time::now().toSec();
    // Abandon our timestamp if its too far off
    if (messageAge > 1.0 || messageAge < -1.0)
    {
      m_navSatFix.header.stamp = ros::Time::now();
      ROS_ERROR("GPS message too old! %f seconds", messageAge);
    }
    
    try
    {
      m_portA.diag("GPS Message Age (s)", (boost::lexical_cast<std::string>(messageAge)).c_str());
    } catch(const boost::bad_lexical_cast &)
    {
      ROS_ERROR_STREAM("GPSReach failed GPS message age lexical cast");
      m_portA.diag_warn("GPSReach failed GPS message age lexical cast");
      return;
    }
    m_statusPub.publish(m_navSatFix);
    m_portA.tick("Publishing navSatFix");
  } else if(msgType == ">JRTK")
  {
    if(tokens.size() < 2)
    {
      ROS_WARN("GPSReach: wrong token count 4 in: %s", msg.c_str());
      return;
    }
    if(tokens[1] == "6")
    {
      if(tokens.size() < 5)
      {
        ROS_WARN("GPSReach: wrong token count 5 in: %s", msg.c_str());
        return;
      }
      std::string timeToGo = tokens[2];
      int readyTransmit = atoi(tokens[3].c_str());
      int transmitting = atoi(tokens[4].c_str());
      if(transmitting > 0)
      {
        m_portA.diag("RTK Corrections:", "transmitting");
        m_portA.diag("RTK Fix:", "SBAS");
        m_portB.OK();
      } else if(readyTransmit > 0)
      {
        m_portB.diag("RTK Corrections:", "ready to transmit");
        m_portB.diag("RTK Fix:", "SBAS");
        m_portB.OK();
      } else
      {
        m_portB.diag("RTK Corrections:", timeToGo + " seconds until ready");
        if(atoi(timeToGo.c_str()) == 299)
        {
          m_portB.diag("RTK Fix:", "none");
          m_portB.ERROR();
        } else
        {
          m_portB.diag("RTK Fix:", "unaugmented");
          m_portB.WARN();
        }
      }
    } else if(tokens[1] == "1")
    {
      //ignore since its a reply
    }
  }
  else if(msgType == "GPGSA" ||
          msgType == "GLGSA" ||
          msgType == "GNGSA")
  {
    if(tokens.size() < 20)
    {
      ROS_WARN("GPSReach: %s too few tokens %lu", msgType.c_str(), tokens.size());
      return;
    }
    std::string gnssId;
    if(tokens[18] == "1")
    {
      gnssId = " GPS";
    } else if(tokens[18] == "2")
    {
      gnssId = " GLONASS";
    } else
    {
      gnssId = " unknown gnssId: " + tokens[18];
    }
    m_portA.tick(msgType + gnssId);
    if( (ros::Time::now()-m_previousCovTime).toSec() > 5.0)
    {
        m_navSatFix.position_covariance_type =
              sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
    //only use this if there isn't a better source of covariance information
    //and the fix is valid
    int satsInUse = 0;
    std::string sats;
    for(int i = 3; i < 15; i++)
    {
      if(!tokens[i].empty())
      {
        ++satsInUse;
        sats += tokens[i];
        sats += " ";
      }
    }
    m_portA.diag(msgType + gnssId + " # satellites used:", std::to_string(satsInUse));
    if(satsInUse > 0)
    {
      m_portA.diag(msgType + gnssId + " satellites used:", sats);
    }
    m_portA.diag(msgType + gnssId + " PDOP-HDOP-VDOP", 
                 tokens[15]+"-"+tokens[16]+"-"+tokens[17]);
    if(m_navSatFix.position_covariance_type <=
       sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED &&
       atof(tokens[2].c_str()) > 1)
    { 
      //choose ideal measurmenet error based on fix type
      double multiplier = m_accuracyRTK;
      if(m_navSatFix.status.status <= sensor_msgs::NavSatStatus::STATUS_FIX)
      {
        multiplier = m_accuracyAutonomous;
      } else if(m_navSatFix.status.status <= sensor_msgs::NavSatStatus::STATUS_SBAS_FIX)
      {
        multiplier = m_accuracyWAAS;
      }
      //use DOP*ideal measurement error for std dev estimates
      //HDOP used for lat and lon
      try
      {
        double val = boost::lexical_cast<double>(tokens[4])*multiplier;
        m_navSatFix.position_covariance[0] = val*val;
        m_navSatFix.position_covariance[4] = val*val;
        //VDOP
        //val = boost::lexical_cast<double>(tokens[5])*multiplier;
        val = boost::lexical_cast<double>(tokens[5])*multiplier;
        m_navSatFix.position_covariance[8] = val*val;
        m_navSatFix.position_covariance_type =
                  sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        m_previousCovTime = ros::Time::now();
      } catch(const boost::bad_lexical_cast &)
      {
        m_portA.diag_error("GPSReach: process of GSA msg cause bad lexical cast for:"
                           + msgType);
        ROS_ERROR_STREAM("GPSReach::process " << msgType << " caused bad lexical cast failed");
        return;
      }
    }
  }  else if(msgType == "GPGST")
  {
    if(tokens.size() < 9)
    {
      ROS_WARN("GPSReach: GPGST partial token count: %lu", tokens.size());
      return;
    }
    m_portA.tick("GPGST");
    if( (ros::Time::now()-m_previousCovTime).toSec() > 5.0)
    {
        m_navSatFix.position_covariance_type =
              sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
    //check to see better variance source is available, and the message has data
    if(m_navSatFix.position_covariance_type <=
       sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN &&
       atof(tokens[1].c_str()) > 100)
    {
      if(tokens.size() < 9)
      {
        ROS_WARN("GPSReach: wrong token count 7 in: %s", msg.c_str());
        return;
      }
      //UTC time
      processUTC(tokens[1], msgType);
      //Token 2 = RMS of std dev of range inputs
      //Token 3 = Standard deviation of semi-major axis of error ellipse, meters
      //Token 4 = Standard deviation of semi-minor axis of error ellipse, meters
      //Token 5 = Error in semi major axis origination, in decimal degrees, true north
      //Std dev of latitude error, in meters
      try
      {
        if(!tokens[6].empty())
        {
          double val = boost::lexical_cast<double>(tokens[6]);
          m_navSatFix.position_covariance[0] = val*val;
          //Std dev of longitude error, in meters
          val = boost::lexical_cast<double>(tokens[7]);
          m_navSatFix.position_covariance[4] = val*val;
          //Std dev of altitude error, in meters
          val = boost::lexical_cast<double>(tokens[8]);
          m_navSatFix.position_covariance[8] = val*val;
          m_navSatFix.position_covariance_type =
                  sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
          m_previousCovTime = ros::Time::now();
        }
      } catch(const boost::bad_lexical_cast &)
      {
        m_portA.diag_error("GPSReach: process GPGST bad lexical cast");
        ROS_ERROR("GPSReach: process GPGST bad lexical cast");
        return;
      }
    }
  } else if(msgType == "GPVTG") //course over ground/ground speed
  {
    m_portA.tick("GPVTG");
  }  else if(msgType == "GPZDA") //detailed UTC time information
  {
    if(tokens.size() < 2)
    {
      ROS_WARN("GPSReach: wrong token count 8 in: %s", msg.c_str());
      return;
    }
    m_portA.tick("GPZDA");
    processUTC(tokens[1], "GPZDA");
    //Token 1 = UTC
    //Token 2 = UTC day
    //Token 3 = UTC month
    //Token 4 = UTC year
    //Token 5 = Local zone hours
    //Token 6 = Local zone minutes
  } else if(msgType == "PSAT")
  {
    if(tokens.size() < 2)
    {
      ROS_WARN("GPSReach: wrong token count 9 in: %s", msg.c_str());
      return;
    }
    m_portA.tick("PSAT");
    if(tokens[1] == "RTKSTAT")
    {
    } else if(tokens[1] == "RTKPROG")
    {
    }
  } else if(msgType == "GPGSV" ||
            msgType == "GLGSV")
  {
    if(tokens.size() < 5) //Minimum message with no satelite info in it.
    {
      ROS_WARN("GPSReach: wrong token count 10 in: %s", msg.c_str());
      return;
    }
    
    try
    {
      int totalMessages = boost::lexical_cast<int>(tokens[1]);
      int messageNumber = boost::lexical_cast<int>(tokens[2]);
      if(m_showGsv)
      {
        char channel[20];
        // Iterate through all of the satellites
        for (size_t i = 4; i <= (tokens.size() - 6); i+=4)
        {
          std::string diagnosticMessage;
          diagnosticMessage += " SS: ";
          diagnosticMessage += tokens[i+3];
          if (tokens[i+3]=="") diagnosticMessage += "NA";
          diagnosticMessage += " EL: ";
          diagnosticMessage += tokens[i+1];
          diagnosticMessage += " AZ: ";
          diagnosticMessage += tokens[i+2];
          diagnosticMessage += " Num: ";
          diagnosticMessage += tokens[i];
          std::string diagnosticLabel = msgType;
          diagnosticLabel += " channel ";
          snprintf(channel,20,"%lu",(messageNumber*4) + ((i-4)/4));
          diagnosticLabel += std::string(channel);
          m_portA.diag(diagnosticLabel,diagnosticMessage);
        }
      }
      if(messageNumber == totalMessages)
      {
        //We got complete info
        m_portA.tick(msgType);
      }
    } catch(const boost::bad_lexical_cast &)
    {
      m_portA.diag_error("GPSReach: process GSV failed");
      ROS_ERROR_STREAM("GPSReach: process " << msgType << " failed");
      return;
    }
  } else if(msgType == "GPGNS" ||
            msgType == "GLGNS"  ||
            msgType == "GNGNS")
  {
    m_portA.tick(msgType);
  }else
  {
    ROS_WARN("GPSReach: received unknown message type:%s", msgType.c_str());
  }
} */

std::string GPSReach::processQuality(const std::string& qual)
{
  m_navSatFix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS +
                               sensor_msgs::NavSatStatus::SERVICE_GLONASS;
  if(qual == "0")
  {
    m_portA.ERROR();
    m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    return "0 - no position";
  } else if(qual == "1")
  {
    m_portA.WARN();
    m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    return "1 - undifferentially corrected";
  } else if(qual == "2")
  {
    m_portA.OK();
    m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
    return "2 - differentially corrected";
  } else if(qual == "4")
  {
    m_portA.OK();
    m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    return "4 - RTK fixed integer converged";
  } else if(qual == "5")
  {
    m_portA.OK();
    m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    return "5 - RTK float converged";
  } else
  {
    return qual + " - unknown";
  }
}

std::string GPSReach::processMode(const std::string& modeIndicator)
{
  m_navSatFix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS +
                               sensor_msgs::NavSatStatus::SERVICE_GLONASS;
  if(modeIndicator == "N")
  {
    m_portA.ERROR();
    m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    return "N - no fix";
  } else if(modeIndicator == "A")
  {
    m_portA.WARN();
    m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    return "A - autonomous (undifferentially corrected)";
  } else if(modeIndicator == "D")
  {
    m_portA.OK();
    m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
    return "D - differentially corrected";
  } else if(modeIndicator == "P")
  {
    m_portA.OK();
    m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
    return "P - precise fix";
  } else if(modeIndicator == "R")
  {
    m_portA.OK();
    m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    return "R - RTK fixed integer converged";
  } else if(modeIndicator == "F")
  {
    m_portA.OK();
    m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    return "F - RTK float converged";
  } else if(modeIndicator == "E")
  {
    m_portA.WARN();
    m_navSatFix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    return "E - dead reckoning";
  } else
  {
    return modeIndicator + " - unknown";
  }
}

double GPSReach::processLatitude(const std::string& lat,
                                     const std::string& latInd)
{
  //std::cout << "lat " << lat << std::endl;
  
  try
  {
    if(latInd == "N")
    {
      return boost::lexical_cast<double>(lat.substr(0,2))+
             boost::lexical_cast<double>(lat.substr(2))/60.0;
    } else
    {
      return -(boost::lexical_cast<double>(lat.substr(0,2))+
               boost::lexical_cast<double>(lat.substr(2))/60.0);
    }
  } catch(const boost::bad_lexical_cast &)
  {
    m_portA.diag_error("GPSReach::processLatitude bad lexical cast");
    ROS_ERROR("GPSReach::processLatitude bad lexical cast");
    return 0.0;
  }
}
double GPSReach::processLongitude(const std::string& lon,
                                      const std::string& lonInd)
{
  //std::cout << "lon " << lon << std::endl;
  
  try
  {
    if(lonInd == "E")
    {
      return boost::lexical_cast<double>(lon.substr(0,3))+
             boost::lexical_cast<double>(lon.substr(3))/60.0;
    } else
    {
      return -(boost::lexical_cast<double>(lon.substr(0,3))+
               boost::lexical_cast<double>(lon.substr(3))/60.0);
    }
  } catch(const boost::bad_lexical_cast &)
  {
    m_portA.diag_error("GPSReach::processLongitude bad lexical cast");
    ROS_ERROR("GPSReach::processLongitude bad lexical cast");
    return 0.0;
  }
}
double GPSReach::processAltitude(const std::string& antAlt,
                                      const std::string& antAltUnits,
                                      const std::string& geodSep,
                                      const std::string& geodSepUnits)
{
  if(antAltUnits == "M" && geodSepUnits == "M")
  {
    try
    {
      return boost::lexical_cast<double>(antAlt) +
             boost::lexical_cast<double>(geodSep);
    } catch(const boost::bad_lexical_cast &)
    {
      m_portA.diag_error("GPSReach::processAltitude bad lexical cast");
      ROS_ERROR("GPSReach::processAltitude bad lexical cast");
    } 
  } else
  {
    m_portA.diag_error("GPSReach: unsupported altitude units: Altitude in " +
                       antAltUnits + ". Geoidal Seperation in " + geodSepUnits + 
                       ". Expected 'M' for both.");
  }
  return 0.0;
}

void GPSReach::processUTC(const std::string& utc, const std::string& source)
{
  if(m_utcSource == source)
  {
    m_timeUTC.header.stamp = ros::Time::now();
    
    try
    {
      int sec = boost::lexical_cast<int>(utc.substr(0,2))*360 +
                boost::lexical_cast<int>(utc.substr(2,2))*60 +
                boost::lexical_cast<int>(utc.substr(4,2));
      int nsec = boost::lexical_cast<int>(utc.substr(7,2))*1000000000;
      m_timeUTC.time_ref = ros::Time(sec+m_secondsToToday, nsec);

      m_utcPub.publish(m_timeUTC);
    } catch(const boost::bad_lexical_cast &)
    {
      m_portA.diag_error("GPSReach::processUTC bad lexical cast");
      ROS_ERROR("GPSReach::processUTC bad lexical cast");
      return;
    }
  }
}

double GPSReach::GetUTC(const std::string& utc)
{
  m_timeUTC.header.stamp = ros::Time::now();
  try
  {
    int sec = boost::lexical_cast<int>(utc.substr(0,2))*3600 +
              boost::lexical_cast<int>(utc.substr(2,2))*60 +
              boost::lexical_cast<int>(utc.substr(4,2));
    double dsec = (float)(boost::lexical_cast<int>(utc.substr(7,2)))/100.0;
    return (double)sec + dsec;
  } catch(const boost::bad_lexical_cast &)
  {
    m_portA.diag_error("GPSReach::getUTC bad lexical cast");
    ROS_ERROR("GPSReach::getUTC bad lexical cast");
    return 0.0;
  }
}

void GPSReach::rtkStatusCallback(const ros::TimerEvent& /*time*/)
{
  //query the current RTK transmission status
  unsigned char cmd[10] = "$JRTK,6\r\n";
  m_portA.writePort(cmd,10);

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
    unsigned char cmd[12] = "$JRTK,1,P\r\n";
    m_portA.writePort(cmd, 12);
  }
  return;
}