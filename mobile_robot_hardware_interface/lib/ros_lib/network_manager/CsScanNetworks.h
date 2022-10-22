#ifndef _ROS_SERVICE_CsScanNetworks_h
#define _ROS_SERVICE_CsScanNetworks_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace network_manager
{

static const char CSSCANNETWORKS[] = "network_manager/CsScanNetworks";

  class CsScanNetworksRequest : public ros::Msg
  {
    public:

    CsScanNetworksRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return CSSCANNETWORKS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class CsScanNetworksResponse : public ros::Msg
  {
    public:
      typedef bool _requestPosted_type;
      _requestPosted_type requestPosted;

    CsScanNetworksResponse():
      requestPosted(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_requestPosted;
      u_requestPosted.real = this->requestPosted;
      *(outbuffer + offset + 0) = (u_requestPosted.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->requestPosted);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_requestPosted;
      u_requestPosted.base = 0;
      u_requestPosted.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->requestPosted = u_requestPosted.real;
      offset += sizeof(this->requestPosted);
     return offset;
    }

    const char * getType(){ return CSSCANNETWORKS; };
    const char * getMD5(){ return "2fcb2e5a10cb911a6333fe4822130901"; };

  };

  class CsScanNetworks {
    public:
    typedef CsScanNetworksRequest Request;
    typedef CsScanNetworksResponse Response;
  };

}
#endif
