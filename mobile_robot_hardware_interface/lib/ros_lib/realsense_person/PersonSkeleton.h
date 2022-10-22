#ifndef _ROS_realsense_person_PersonSkeleton_h
#define _ROS_realsense_person_PersonSkeleton_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "realsense_person/RegisteredPoint.h"

namespace realsense_person
{

  class PersonSkeleton : public ros::Msg
  {
    public:
      typedef const char* _joint_type_type;
      _joint_type_type joint_type;
      typedef realsense_person::RegisteredPoint _skeleton_point_type;
      _skeleton_point_type skeleton_point;

    PersonSkeleton():
      joint_type(""),
      skeleton_point()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_joint_type = strlen(this->joint_type);
      varToArr(outbuffer + offset, length_joint_type);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_type, length_joint_type);
      offset += length_joint_type;
      offset += this->skeleton_point.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_joint_type;
      arrToVar(length_joint_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joint_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joint_type-1]=0;
      this->joint_type = (char *)(inbuffer + offset-1);
      offset += length_joint_type;
      offset += this->skeleton_point.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "realsense_person/PersonSkeleton"; };
    const char * getMD5(){ return "830cfdb700931a76a5884c7211cc3672"; };

  };

}
#endif