#ifndef _ROS_realsense_person_RegisteredPoint_h
#define _ROS_realsense_person_RegisteredPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"

namespace realsense_person
{

  class RegisteredPoint : public ros::Msg
  {
    public:
      typedef geometry_msgs::Point _world_type;
      _world_type world;
      typedef int32_t _world_confidence_type;
      _world_confidence_type world_confidence;
      typedef geometry_msgs::Point _image_type;
      _image_type image;
      typedef int32_t _image_confidence_type;
      _image_confidence_type image_confidence;

    RegisteredPoint():
      world(),
      world_confidence(0),
      image(),
      image_confidence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->world.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_world_confidence;
      u_world_confidence.real = this->world_confidence;
      *(outbuffer + offset + 0) = (u_world_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_world_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_world_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_world_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->world_confidence);
      offset += this->image.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_image_confidence;
      u_image_confidence.real = this->image_confidence;
      *(outbuffer + offset + 0) = (u_image_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_image_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_image_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_image_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->image_confidence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->world.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_world_confidence;
      u_world_confidence.base = 0;
      u_world_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_world_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_world_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_world_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->world_confidence = u_world_confidence.real;
      offset += sizeof(this->world_confidence);
      offset += this->image.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_image_confidence;
      u_image_confidence.base = 0;
      u_image_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_image_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_image_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_image_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->image_confidence = u_image_confidence.real;
      offset += sizeof(this->image_confidence);
     return offset;
    }

    const char * getType(){ return "realsense_person/RegisteredPoint"; };
    const char * getMD5(){ return "377e1b201e51fdf9a771993c3be57884"; };

  };

}
#endif