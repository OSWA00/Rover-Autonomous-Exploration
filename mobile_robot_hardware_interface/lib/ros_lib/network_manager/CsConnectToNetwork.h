#ifndef _ROS_SERVICE_CsConnectToNetwork_h
#define _ROS_SERVICE_CsConnectToNetwork_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace network_manager
{

static const char CSCONNECTTONETWORK[] = "network_manager/CsConnectToNetwork";

  class CsConnectToNetworkRequest : public ros::Msg
  {
    public:
      typedef const char* _ssid_type;
      _ssid_type ssid;
      typedef const char* _password_type;
      _password_type password;

    CsConnectToNetworkRequest():
      ssid(""),
      password("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_ssid = strlen(this->ssid);
      varToArr(outbuffer + offset, length_ssid);
      offset += 4;
      memcpy(outbuffer + offset, this->ssid, length_ssid);
      offset += length_ssid;
      uint32_t length_password = strlen(this->password);
      varToArr(outbuffer + offset, length_password);
      offset += 4;
      memcpy(outbuffer + offset, this->password, length_password);
      offset += length_password;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_ssid;
      arrToVar(length_ssid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ssid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ssid-1]=0;
      this->ssid = (char *)(inbuffer + offset-1);
      offset += length_ssid;
      uint32_t length_password;
      arrToVar(length_password, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_password; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_password-1]=0;
      this->password = (char *)(inbuffer + offset-1);
      offset += length_password;
     return offset;
    }

    const char * getType(){ return CSCONNECTTONETWORK; };
    const char * getMD5(){ return "be0ef5217dea86e981988b8b0821adc4"; };

  };

  class CsConnectToNetworkResponse : public ros::Msg
  {
    public:
      typedef bool _res_type;
      _res_type res;
      typedef const char* _message_type;
      _message_type message;

    CsConnectToNetworkResponse():
      res(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_res;
      u_res.real = this->res;
      *(outbuffer + offset + 0) = (u_res.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->res);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_res;
      u_res.base = 0;
      u_res.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->res = u_res.real;
      offset += sizeof(this->res);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
     return offset;
    }

    const char * getType(){ return CSCONNECTTONETWORK; };
    const char * getMD5(){ return "9b4df51719804cbf1b97591437594b37"; };

  };

  class CsConnectToNetwork {
    public:
    typedef CsConnectToNetworkRequest Request;
    typedef CsConnectToNetworkResponse Response;
  };

}
#endif
