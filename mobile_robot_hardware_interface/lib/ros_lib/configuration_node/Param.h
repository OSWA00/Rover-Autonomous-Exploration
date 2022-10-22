#ifndef _ROS_configuration_node_Param_h
#define _ROS_configuration_node_Param_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

  class Param : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _val_type;
      _val_type val;

    Param():
      name(""),
      val("")
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
      uint32_t length_val = strlen(this->val);
      varToArr(outbuffer + offset, length_val);
      offset += 4;
      memcpy(outbuffer + offset, this->val, length_val);
      offset += length_val;
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
      uint32_t length_val;
      arrToVar(length_val, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_val; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_val-1]=0;
      this->val = (char *)(inbuffer + offset-1);
      offset += length_val;
     return offset;
    }

    const char * getType(){ return "configuration_node/Param"; };
    const char * getMD5(){ return "672fe963919628c6048bd6f60445c089"; };

  };

}
#endif