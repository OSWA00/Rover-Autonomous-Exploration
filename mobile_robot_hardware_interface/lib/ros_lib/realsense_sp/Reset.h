#ifndef _ROS_SERVICE_Reset_h
#define _ROS_SERVICE_Reset_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_sp
{

static const char RESET[] = "realsense_sp/Reset";

  class ResetRequest : public ros::Msg
  {
    public:

    ResetRequest()
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

    const char * getType(){ return RESET; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ResetResponse : public ros::Msg
  {
    public:

    ResetResponse()
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

    const char * getType(){ return RESET; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Reset {
    public:
    typedef ResetRequest Request;
    typedef ResetResponse Response;
  };

}
#endif
