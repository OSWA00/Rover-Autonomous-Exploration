#ifndef _ROS_realsense_person_PersonFace_h
#define _ROS_realsense_person_PersonFace_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "realsense_person/BoundingBox.h"
#include "geometry_msgs/Point.h"
#include "realsense_person/RegisteredPoint.h"

namespace realsense_person
{

  class PersonFace : public ros::Msg
  {
    public:
      typedef const char* _orientation_type;
      _orientation_type orientation;
      typedef int32_t _orientation_confidence_type;
      _orientation_confidence_type orientation_confidence;
      typedef realsense_person::BoundingBox _head_bounding_box_type;
      _head_bounding_box_type head_bounding_box;
      typedef geometry_msgs::Point _head_pose_type;
      _head_pose_type head_pose;
      uint32_t landmarks_length;
      typedef realsense_person::RegisteredPoint _landmarks_type;
      _landmarks_type st_landmarks;
      _landmarks_type * landmarks;
      typedef int32_t _landmarks_confidence_type;
      _landmarks_confidence_type landmarks_confidence;

    PersonFace():
      orientation(""),
      orientation_confidence(0),
      head_bounding_box(),
      head_pose(),
      landmarks_length(0), landmarks(NULL),
      landmarks_confidence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_orientation = strlen(this->orientation);
      varToArr(outbuffer + offset, length_orientation);
      offset += 4;
      memcpy(outbuffer + offset, this->orientation, length_orientation);
      offset += length_orientation;
      union {
        int32_t real;
        uint32_t base;
      } u_orientation_confidence;
      u_orientation_confidence.real = this->orientation_confidence;
      *(outbuffer + offset + 0) = (u_orientation_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_orientation_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_orientation_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_orientation_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->orientation_confidence);
      offset += this->head_bounding_box.serialize(outbuffer + offset);
      offset += this->head_pose.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->landmarks_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->landmarks_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->landmarks_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->landmarks_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->landmarks_length);
      for( uint32_t i = 0; i < landmarks_length; i++){
      offset += this->landmarks[i].serialize(outbuffer + offset);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_landmarks_confidence;
      u_landmarks_confidence.real = this->landmarks_confidence;
      *(outbuffer + offset + 0) = (u_landmarks_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_landmarks_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_landmarks_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_landmarks_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->landmarks_confidence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_orientation;
      arrToVar(length_orientation, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_orientation; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_orientation-1]=0;
      this->orientation = (char *)(inbuffer + offset-1);
      offset += length_orientation;
      union {
        int32_t real;
        uint32_t base;
      } u_orientation_confidence;
      u_orientation_confidence.base = 0;
      u_orientation_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_orientation_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_orientation_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_orientation_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->orientation_confidence = u_orientation_confidence.real;
      offset += sizeof(this->orientation_confidence);
      offset += this->head_bounding_box.deserialize(inbuffer + offset);
      offset += this->head_pose.deserialize(inbuffer + offset);
      uint32_t landmarks_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      landmarks_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      landmarks_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      landmarks_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->landmarks_length);
      if(landmarks_lengthT > landmarks_length)
        this->landmarks = (realsense_person::RegisteredPoint*)realloc(this->landmarks, landmarks_lengthT * sizeof(realsense_person::RegisteredPoint));
      landmarks_length = landmarks_lengthT;
      for( uint32_t i = 0; i < landmarks_length; i++){
      offset += this->st_landmarks.deserialize(inbuffer + offset);
        memcpy( &(this->landmarks[i]), &(this->st_landmarks), sizeof(realsense_person::RegisteredPoint));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_landmarks_confidence;
      u_landmarks_confidence.base = 0;
      u_landmarks_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_landmarks_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_landmarks_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_landmarks_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->landmarks_confidence = u_landmarks_confidence.real;
      offset += sizeof(this->landmarks_confidence);
     return offset;
    }

    const char * getType(){ return "realsense_person/PersonFace"; };
    const char * getMD5(){ return "3a9e6d7f59e4f091518a51ac905c4d3f"; };

  };

}
#endif