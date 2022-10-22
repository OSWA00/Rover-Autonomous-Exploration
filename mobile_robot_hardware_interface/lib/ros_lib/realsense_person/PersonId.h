#ifndef _ROS_realsense_person_PersonId_h
#define _ROS_realsense_person_PersonId_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_person
{

  class PersonId : public ros::Msg
  {
    public:
      typedef uint32_t _tracking_id_type;
      _tracking_id_type tracking_id;
      typedef uint32_t _recognition_id_type;
      _recognition_id_type recognition_id;
      typedef const char* _recognition_name_type;
      _recognition_name_type recognition_name;

    PersonId():
      tracking_id(0),
      recognition_id(0),
      recognition_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->tracking_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tracking_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tracking_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tracking_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tracking_id);
      *(outbuffer + offset + 0) = (this->recognition_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->recognition_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->recognition_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->recognition_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->recognition_id);
      uint32_t length_recognition_name = strlen(this->recognition_name);
      varToArr(outbuffer + offset, length_recognition_name);
      offset += 4;
      memcpy(outbuffer + offset, this->recognition_name, length_recognition_name);
      offset += length_recognition_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->tracking_id =  ((uint32_t) (*(inbuffer + offset)));
      this->tracking_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tracking_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tracking_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tracking_id);
      this->recognition_id =  ((uint32_t) (*(inbuffer + offset)));
      this->recognition_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->recognition_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->recognition_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->recognition_id);
      uint32_t length_recognition_name;
      arrToVar(length_recognition_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_recognition_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_recognition_name-1]=0;
      this->recognition_name = (char *)(inbuffer + offset-1);
      offset += length_recognition_name;
     return offset;
    }

    const char * getType(){ return "realsense_person/PersonId"; };
    const char * getMD5(){ return "15a47b5998bee60eae12041bea9a93f8"; };

  };

}
#endif