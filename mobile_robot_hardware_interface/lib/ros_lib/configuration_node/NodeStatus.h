#ifndef _ROS_configuration_node_NodeStatus_h
#define _ROS_configuration_node_NodeStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

  class NodeStatus : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef bool _isRunning_type;
      _isRunning_type isRunning;

    NodeStatus():
      name(""),
      isRunning(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      union {
        bool real;
        uint8_t base;
      } u_isRunning;
      u_isRunning.real = this->isRunning;
      *(outbuffer + offset + 0) = (u_isRunning.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isRunning);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      union {
        bool real;
        uint8_t base;
      } u_isRunning;
      u_isRunning.base = 0;
      u_isRunning.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isRunning = u_isRunning.real;
      offset += sizeof(this->isRunning);
     return offset;
    }

    const char * getType(){ return "configuration_node/NodeStatus"; };
    const char * getMD5(){ return "520540f797a12cf677401db2fbca3927"; };

  };

}
#endif