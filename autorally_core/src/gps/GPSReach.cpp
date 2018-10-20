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

  if(!m_portA.connected() /*|| !m_portB.connected()*/)
  {
    ROS_ERROR("GPSReach: the primary serial port isn't open, stuff might not work");
  }

  if (m_bIsBase && !m_portB.connected())
  {
    ROS_ERROR("GPSReach: the correction port is not open, correction data will not be available.");
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
          ROS_DEBUG_STREAM("GPSReach:: Invalid Checksum  for type: " << (int) _msg_id);
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
          ROS_DEBUG_STREAM("GPSReach:: Invalid Checksum for type: " << (int) _msg_id);
          continue;
        }

        processGPSMessage(_msg_id);
        ROS_DEBUG_STREAM("GPSReach:: Successfully parsed message of type: " << (int) _msg_id); 
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

  ROS_DEBUG_STREAM("GPS Time Raw: " << millisecs << " week: " << week);
  ROS_DEBUG_STREAM("GPS Time " << asctime(gmtime(&m_gpsTime)));

  if (m_bIsBase)
  {
      m_timeUTC.header.stamp = ros::Time::now();
      m_timeUTC.time_ref = ros::Time(m_gpsTime);
      m_utcPub.publish(m_timeUTC);
  }
}

void GPSReach::processGPSMessage(int msgId)
{
  switch(msgId)
  {
    case MSG_VER:
      //erb_ver ver = (erb_ver) buffer;
      ROS_DEBUG_STREAM("ERB Version: Time: " << _buffer.ver.time << " Ver: " <<  (int) _buffer.ver.ver_high << ":" << (int) _buffer.ver.ver_medium << ":" << (int)_buffer.ver.ver_low);
      break;
    case MSG_DOPS:
      ROS_DEBUG_STREAM("DOPS: " << (int) _buffer.dops.hDOP);
      break;
    case MSG_STAT:
      ROS_DEBUG_STREAM("GPS Status: Fix Type: " << (int) _buffer.stat.fix_type << "Fix Status: " << (int) _buffer.stat.fix_status << " Sats: " << (int) _buffer.stat.satellites << " Time: " << _buffer.stat.time << " Week: " << _buffer.stat.week);
      break;
    case MSG_POS:
      ROS_DEBUG_STREAM("GPS Pos: Long: " << _buffer.pos.longitude << " Lat: " << _buffer.pos.latitude << " Alt: " << _buffer.pos.altitude_msl);
      break;
    case MSG_RTK:
      ROS_DEBUG_STREAM("RTK Data");
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
    ROS_DEBUG_STREAM("Process RTK");
    //publishRTCMData();
  }
}

void GPSReach::publishRTCMData()
{  
  m_portB.lock();

  if(m_portB.m_data.size() > 6)
  {
    //make sure data is framed
    if((m_portB.m_data[0]&0xff) != 0xd3)
    {
      std::string s;
      s.push_back(0xd3);
      s.push_back(0x00);
      size_t start = m_portB.m_data.find(s);
      //ROS_WARN_STREAM("Not Framed");
      //std::cout << "Discarding:" << m_portB.m_data.substr(0, start).size() <<
      //  " Leading with:" << (unsigned int)(m_portB.m_data[0]&0xff) << std::endl;
      //printMessage(m_portB.m_data);
      m_portB.m_data.erase(0, start);
    }

    //process if there is enough data to read the header and message type
    if(m_portB.m_data.length() > 6)
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
      unsigned int len = (unsigned int)((((unsigned int)m_portB.m_data[1])&0x03)>>8) +
                         (unsigned int)(((m_portB.m_data[2])&0xff)) + 6;
      unsigned int type = (unsigned int)(((m_portB.m_data[3])&0xff)<<4) +
                          (unsigned int)(((m_portB.m_data[4])&0xf0)>>4);

      /*printf("%x %x %x %x %x\n", m_portB.m_data[0]&0xff,
                                 m_portB.m_data[1]&0xff,
                                 m_portB.m_data[2]&0xff,
                                 m_portB.m_data[3]&0xff,
                                 m_portB.m_data[4]&0xff);*/

      if(m_portB.m_data.size() >= len && ((type > 1000 && type < 1030) || (type > 4087 && type <= 4096)))
      {
        //printMessage(m_portB.m_data);
        //ROS_WARN_STREAM("Buffer len:" << m_portB.m_data.length() <<
        //                " Payload len:" << len << " msg type:" << type);
        //record type of message seen in diagnostics
        try
        {
          m_portB.tick("RTCM3.0 type "+boost::lexical_cast<std::string>(type));
         
          //fill in structure to send message
          m_rtkCorrection.layout.dim.front().label = "RTCM3.0 " +
                          boost::lexical_cast<std::string>(type);
      
          m_rtkCorrection.layout.dim.front().size = len;
          m_rtkCorrection.layout.dim.front().stride = 1*(len);
          m_rtkCorrection.data.resize(len);
          memcpy(&m_rtkCorrection.data[0], &m_portB.m_data[0], len);

          m_rtcm3Pub.publish(m_rtkCorrection);
        } catch(const boost::bad_lexical_cast &)
        {
          ROS_ERROR_STREAM("GPSReach failed RTCM3.0 type lexical cast:" << type);
          m_portB.diag_warn("GPSReach failed RTCM3.0 type lexical cast");
          return;
        }
        
        m_portB.m_data.erase(0,len);
        //std::cout << "\t new B Len:" << m_portB.m_data.length() << std::endl;
      } else if(m_portB.m_data.size() >= len)
      {
        m_portB.m_data.erase(0,len);
        ROS_WARN_STREAM("GPSReach:: unknown RTCM3.0 message type:" << type << " of length:" << len);
        m_portB.diag_warn("GPSReach:: unknown " + m_rtkCorrection.layout.dim.front().label);
      }

    }
  }

  m_portB.unlock();
}

void GPSReach::rtcmCorrectionCallback(const std_msgs::ByteMultiArray& msg)
{
  ROS_DEBUG_STREAM("write RTCM3 correction data");
  m_portA.lock();
  m_portA.tick("Incoming Correction Data");
  m_mostRecentRTK = ros::Time::now();
  m_portA.writePort( reinterpret_cast<const unsigned char*>(&msg.data[0]),
                     msg.layout.dim[0].size);
  m_portA.unlock();
}

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
