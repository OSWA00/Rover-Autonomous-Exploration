#ifndef _ROS_realsense_person_Person_h
#define _ROS_realsense_person_Person_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "realsense_person/PersonId.h"
#include "realsense_person/BoundingBox.h"
#include "realsense_person/RegisteredPoint.h"

namespace realsense_person
{

  class Person : public ros::Msg
  {
    public:
      typedef realsense_person::PersonId _person_id_type;
      _person_id_type person_id;
      typedef realsense_person::BoundingBox _bounding_box_type;
      _bounding_box_type bounding_box;
      typedef realsense_person::RegisteredPoint _center_of_mass_type;
      _center_of_mass_type center_of_mass;

    Person():
      person_id(),
      bounding_box(),
      center_of_mass()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->person_id.serialize(outbuffer + offset);
      offset += this->bounding_box.serialize(outbuffer + offset);
      offset += this->center_of_mass.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->person_id.deserialize(inbuffer + offset);
      offset += this->bounding_box.deserialize(inbuffer + offset);
      offset += this->center_of_mass.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "realsense_person/Person"; };
    const char * getMD5(){ return "32ab981d8916bb2d20cd1462aba7836c"; };

  };

}
#endif