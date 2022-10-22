#ifndef _ROS_SERVICE_ReloadEuclidNode_h
#define _ROS_SERVICE_ReloadEuclidNode_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

static const char RELOADEUCLIDNODE[] = "configuration_node/ReloadEuclidNode";

  class ReloadEuclidNodeRequest : public ros::Msg
  {
    public:
      typedef const char* _nodeName_type;
      _nodeName_type nodeName;

    ReloadEuclidNodeRequest():
      nodeName("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_nodeName = strlen(this->nodeName);
      varToArr(outbuffer + offset, length_nodeName);
      offset += 4;
      memcpy(outbuffer + offset, this->nodeName, length_nodeName);
      offset += length_nodeName;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_nodeName;
      arrToVar(length_nodeName, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_nodeName; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_nodeName-1]=0;
      this->nodeName = (char *)(inbuffer + offset-1);
      offset += length_nodeName;
     return offset;
    }

    const char * getType(){ return RELOADEUCLIDNODE; };
    const char * getMD5(){ return "fa90f177c62ffd4f5a57999c98f4104f"; };

  };

  class ReloadEuclidNodeResponse : public ros::Msg
  {
    public:
      typedef bool _res_type;
      _res_type res;

    ReloadEuclidNodeResponse():
      res(0)
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
     return offset;
    }

    const char * getType(){ return RELOADEUCLIDNODE; };
    const char * getMD5(){ return "e27848a10f8e7e4030443887dfea101b"; };

  };

  class ReloadEuclidNode {
    public:
    typedef ReloadEuclidNodeRequest Request;
    typedef ReloadEuclidNodeResponse Response;
  };

}
#endif
