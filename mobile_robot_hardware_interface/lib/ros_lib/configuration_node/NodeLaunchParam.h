#ifndef _ROS_configuration_node_NodeLaunchParam_h
#define _ROS_configuration_node_NodeLaunchParam_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

  class NodeLaunchParam : public ros::Msg
  {
    public:
      typedef const char* _node_type;
      _node_type node;
      typedef const char* _param_type;
      _param_type param;

    NodeLaunchParam():
      node(""),
      param("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_node = strlen(this->node);
      varToArr(outbuffer + offset, length_node);
      offset += 4;
      memcpy(outbuffer + offset, this->node, length_node);
      offset += length_node;
      uint32_t length_param = strlen(this->param);
      varToArr(outbuffer + offset, length_param);
      offset += 4;
      memcpy(outbuffer + offset, this->param, length_param);
      offset += length_param;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_node;
      arrToVar(length_node, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_node; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_node-1]=0;
      this->node = (char *)(inbuffer + offset-1);
      offset += length_node;
      uint32_t length_param;
      arrToVar(length_param, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_param; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_param-1]=0;
      this->param = (char *)(inbuffer + offset-1);
      offset += length_param;
     return offset;
    }

    const char * getType(){ return "configuration_node/NodeLaunchParam"; };
    const char * getMD5(){ return "7eb91335c5be76bb26be823fc9f131f5"; };

  };

}
#endif