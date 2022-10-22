#ifndef _ROS_realsense_person_PersonBody_h
#define _ROS_realsense_person_PersonBody_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "realsense_person/PersonSkeleton.h"
#include "realsense_person/RegisteredPoint.h"

namespace realsense_person
{

  class PersonBody : public ros::Msg
  {
    public:
      uint32_t skeleton_length;
      typedef realsense_person::PersonSkeleton _skeleton_type;
      _skeleton_type st_skeleton;
      _skeleton_type * skeleton;
      typedef realsense_person::RegisteredPoint _gesture_origin_type;
      _gesture_origin_type gesture_origin;
      typedef realsense_person::RegisteredPoint _gesture_direction_type;
      _gesture_direction_type gesture_direction;
      typedef int32_t _gesture_confidence_type;
      _gesture_confidence_type gesture_confidence;

    PersonBody():
      skeleton_length(0), skeleton(NULL),
      gesture_origin(),
      gesture_direction(),
      gesture_confidence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->skeleton_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->skeleton_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->skeleton_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->skeleton_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->skeleton_length);
      for( uint32_t i = 0; i < skeleton_length; i++){
      offset += this->skeleton[i].serialize(outbuffer + offset);
      }
      offset += this->gesture_origin.serialize(outbuffer + offset);
      offset += this->gesture_direction.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_gesture_confidence;
      u_gesture_confidence.real = this->gesture_confidence;
      *(outbuffer + offset + 0) = (u_gesture_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gesture_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gesture_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gesture_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gesture_confidence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t skeleton_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      skeleton_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      skeleton_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      skeleton_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->skeleton_length);
      if(skeleton_lengthT > skeleton_length)
        this->skeleton = (realsense_person::PersonSkeleton*)realloc(this->skeleton, skeleton_lengthT * sizeof(realsense_person::PersonSkeleton));
      skeleton_length = skeleton_lengthT;
      for( uint32_t i = 0; i < skeleton_length; i++){
      offset += this->st_skeleton.deserialize(inbuffer + offset);
        memcpy( &(this->skeleton[i]), &(this->st_skeleton), sizeof(realsense_person::PersonSkeleton));
      }
      offset += this->gesture_origin.deserialize(inbuffer + offset);
      offset += this->gesture_direction.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_gesture_confidence;
      u_gesture_confidence.base = 0;
      u_gesture_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gesture_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gesture_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gesture_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gesture_confidence = u_gesture_confidence.real;
      offset += sizeof(this->gesture_confidence);
     return offset;
    }

    const char * getType(){ return "realsense_person/PersonBody"; };
    const char * getMD5(){ return "a9bfd5f8f67d8fbcfe06054caaec91ff"; };

  };

}
#endif