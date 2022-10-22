#ifndef _ROS_SERVICE_DeleteNode_h
#define _ROS_SERVICE_DeleteNode_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

static const char DELETENODE[] = "configuration_node/DeleteNode";

  class DeleteNodeRequest : public ros::Msg
  {
    public:
      typedef const char* _nodeName_type;
      _nodeName_type nodeName;

    DeleteNodeRequest():
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

    const char * getType(){ return DELETENODE; };
    const char * getMD5(){ return "fa90f177c62ffd4f5a57999c98f4104f"; };

  };

  class DeleteNodeResponse : public ros::Msg
  {
    public:
      typedef int8_t _ret_type;
      _ret_type ret;
      typedef bool _res_type;
      _res_type res;

    DeleteNodeResponse():
      ret(0),
      res(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_ret;
      u_ret.real = this->ret;
      *(outbuffer + offset + 0) = (u_ret.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ret);
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
        int8_t real;
        uint8_t base;
      } u_ret;
      u_ret.base = 0;
      u_ret.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ret = u_ret.real;
      offset += sizeof(this->ret);
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

    const char * getType(){ return DELETENODE; };
    const char * getMD5(){ return "fd81ec3bb1afe3bab4df26b3ee085f2b"; };

  };

  class DeleteNode {
    public:
    typedef DeleteNodeRequest Request;
    typedef DeleteNodeResponse Response;
  };

}
#endif
