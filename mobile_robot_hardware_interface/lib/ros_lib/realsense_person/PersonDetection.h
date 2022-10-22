#ifndef _ROS_realsense_person_PersonDetection_h
#define _ROS_realsense_person_PersonDetection_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "realsense_person/Person.h"

namespace realsense_person
{

  class PersonDetection : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _detected_person_count_type;
      _detected_person_count_type detected_person_count;
      uint32_t persons_length;
      typedef realsense_person::Person _persons_type;
      _persons_type st_persons;
      _persons_type * persons;

    PersonDetection():
      header(),
      detected_person_count(0),
      persons_length(0), persons(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->detected_person_count >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->detected_person_count >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->detected_person_count >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->detected_person_count >> (8 * 3)) & 0xFF;
      offset += sizeof(this->detected_person_count);
      *(outbuffer + offset + 0) = (this->persons_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->persons_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->persons_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->persons_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->persons_length);
      for( uint32_t i = 0; i < persons_length; i++){
      offset += this->persons[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->detected_person_count =  ((uint32_t) (*(inbuffer + offset)));
      this->detected_person_count |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->detected_person_count |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->detected_person_count |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->detected_person_count);
      uint32_t persons_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      persons_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      persons_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      persons_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->persons_length);
      if(persons_lengthT > persons_length)
        this->persons = (realsense_person::Person*)realloc(this->persons, persons_lengthT * sizeof(realsense_person::Person));
      persons_length = persons_lengthT;
      for( uint32_t i = 0; i < persons_length; i++){
      offset += this->st_persons.deserialize(inbuffer + offset);
        memcpy( &(this->persons[i]), &(this->st_persons), sizeof(realsense_person::Person));
      }
     return offset;
    }

    const char * getType(){ return "realsense_person/PersonDetection"; };
    const char * getMD5(){ return "57cbc8b79ff349e7bf327ad4062aba61"; };

  };

}
#endif