#ifndef _ROS_realsense_person_PersonTracking_h
#define _ROS_realsense_person_PersonTracking_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "realsense_person/Person.h"
#include "realsense_person/PersonFace.h"
#include "realsense_person/PersonBody.h"

namespace realsense_person
{

  class PersonTracking : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef realsense_person::Person _person_type;
      _person_type person;
      typedef realsense_person::PersonFace _person_face_type;
      _person_face_type person_face;
      typedef realsense_person::PersonBody _person_body_type;
      _person_body_type person_body;

    PersonTracking():
      header(),
      person(),
      person_face(),
      person_body()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->person.serialize(outbuffer + offset);
      offset += this->person_face.serialize(outbuffer + offset);
      offset += this->person_body.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->person.deserialize(inbuffer + offset);
      offset += this->person_face.deserialize(inbuffer + offset);
      offset += this->person_body.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "realsense_person/PersonTracking"; };
    const char * getMD5(){ return "7c5085a50c0b109ec84e7a6c91228362"; };

  };

}
#endif