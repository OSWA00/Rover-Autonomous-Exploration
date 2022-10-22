#ifndef _ROS_SERVICE_CsGetCurrentConnectionName_h
#define _ROS_SERVICE_CsGetCurrentConnectionName_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace network_manager
{

static const char CSGETCURRENTCONNECTIONNAME[] = "network_manager/CsGetCurrentConnectionName";

  class CsGetCurrentConnectionNameRequest : public ros::Msg
  {
    public:

    CsGetCurrentConnectionNameRequest()
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

    const char * getType(){ return CSGETCURRENTCONNECTIONNAME; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class CsGetCurrentConnectionNameResponse : public ros::Msg
  {
    public:
      typedef const char* _currentConnection_type;
      _currentConnection_type currentConnection;

    CsGetCurrentConnectionNameResponse():
      currentConnection("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_currentConnection = strlen(this->currentConnection);
      varToArr(outbuffer + offset, length_currentConnection);
      offset += 4;
      memcpy(outbuffer + offset, this->currentConnection, length_currentConnection);
      offset += length_currentConnection;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_currentConnection;
      arrToVar(length_currentConnection, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_currentConnection; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_currentConnection-1]=0;
      this->currentConnection = (char *)(inbuffer + offset-1);
      offset += length_currentConnection;
     return offset;
    }

    const char * getType(){ return CSGETCURRENTCONNECTIONNAME; };
    const char * getMD5(){ return "356a88135e78c0d5fea21d78faf0432e"; };

  };

  class CsGetCurrentConnectionName {
    public:
    typedef CsGetCurrentConnectionNameRequest Request;
    typedef CsGetCurrentConnectionNameResponse Response;
  };

}
#endif
