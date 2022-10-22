#ifndef _ROS_SERVICE_GetTrackingId_h
#define _ROS_SERVICE_GetTrackingId_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_person
{

static const char GETTRACKINGID[] = "realsense_person/GetTrackingId";

  class GetTrackingIdRequest : public ros::Msg
  {
    public:

    GetTrackingIdRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETTRACKINGID; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetTrackingIdResponse : public ros::Msg
  {
    public:
      typedef uint32_t _detected_person_count_type;
      _detected_person_count_type detected_person_count;
      uint32_t tracking_ids_length;
      typedef uint32_t _tracking_ids_type;
      _tracking_ids_type st_tracking_ids;
      _tracking_ids_type * tracking_ids;

    GetTrackingIdResponse():
      detected_person_count(0),
      tracking_ids_length(0), tracking_ids(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->detected_person_count >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->detected_person_count >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->detected_person_count >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->detected_person_count >> (8 * 3)) & 0xFF;
      offset += sizeof(this->detected_person_count);
      *(outbuffer + offset + 0) = (this->tracking_ids_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tracking_ids_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tracking_ids_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tracking_ids_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tracking_ids_length);
      for( uint32_t i = 0; i < tracking_ids_length; i++){
      *(outbuffer + offset + 0) = (this->tracking_ids[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tracking_ids[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tracking_ids[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tracking_ids[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tracking_ids[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->detected_person_count =  ((uint32_t) (*(inbuffer + offset)));
      this->detected_person_count |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->detected_person_count |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->detected_person_count |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->detected_person_count);
      uint32_t tracking_ids_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tracking_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tracking_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tracking_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tracking_ids_length);
      if(tracking_ids_lengthT > tracking_ids_length)
        this->tracking_ids = (uint32_t*)realloc(this->tracking_ids, tracking_ids_lengthT * sizeof(uint32_t));
      tracking_ids_length = tracking_ids_lengthT;
      for( uint32_t i = 0; i < tracking_ids_length; i++){
      this->st_tracking_ids =  ((uint32_t) (*(inbuffer + offset)));
      this->st_tracking_ids |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_tracking_ids |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_tracking_ids |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_tracking_ids);
        memcpy( &(this->tracking_ids[i]), &(this->st_tracking_ids), sizeof(uint32_t));
      }
     return offset;
    }

    const char * getType(){ return GETTRACKINGID; };
    const char * getMD5(){ return "7e485f0e095e33bcc70d0844d2d2cfc2"; };

  };

  class GetTrackingId {
    public:
    typedef GetTrackingIdRequest Request;
    typedef GetTrackingIdResponse Response;
  };

}
#endif
