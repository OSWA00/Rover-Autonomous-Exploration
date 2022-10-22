#ifndef _ROS_SERVICE_Recognize_h
#define _ROS_SERVICE_Recognize_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_person
{

static const char RECOGNIZE[] = "realsense_person/Recognize";

  class RecognizeRequest : public ros::Msg
  {
    public:
      typedef int32_t _tracking_id_type;
      _tracking_id_type tracking_id;

    RecognizeRequest():
      tracking_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_tracking_id;
      u_tracking_id.real = this->tracking_id;
      *(outbuffer + offset + 0) = (u_tracking_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tracking_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tracking_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tracking_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tracking_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_tracking_id;
      u_tracking_id.base = 0;
      u_tracking_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tracking_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tracking_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tracking_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tracking_id = u_tracking_id.real;
      offset += sizeof(this->tracking_id);
     return offset;
    }

    const char * getType(){ return RECOGNIZE; };
    const char * getMD5(){ return "c32022e1bd9eb984e3f9824e577a61b3"; };

  };

  class RecognizeResponse : public ros::Msg
  {
    public:
      typedef int32_t _status_type;
      _status_type status;
      typedef const char* _status_desc_type;
      _status_desc_type status_desc;
      typedef int32_t _recognition_id_type;
      _recognition_id_type recognition_id;
      typedef float _confidence_type;
      _confidence_type confidence;

    RecognizeResponse():
      status(0),
      status_desc(""),
      recognition_id(0),
      confidence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_status.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_status.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status);
      uint32_t length_status_desc = strlen(this->status_desc);
      varToArr(outbuffer + offset, length_status_desc);
      offset += 4;
      memcpy(outbuffer + offset, this->status_desc, length_status_desc);
      offset += length_status_desc;
      union {
        int32_t real;
        uint32_t base;
      } u_recognition_id;
      u_recognition_id.real = this->recognition_id;
      *(outbuffer + offset + 0) = (u_recognition_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_recognition_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_recognition_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_recognition_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->recognition_id);
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.real = this->confidence;
      *(outbuffer + offset + 0) = (u_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->status = u_status.real;
      offset += sizeof(this->status);
      uint32_t length_status_desc;
      arrToVar(length_status_desc, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status_desc; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status_desc-1]=0;
      this->status_desc = (char *)(inbuffer + offset-1);
      offset += length_status_desc;
      union {
        int32_t real;
        uint32_t base;
      } u_recognition_id;
      u_recognition_id.base = 0;
      u_recognition_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_recognition_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_recognition_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_recognition_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->recognition_id = u_recognition_id.real;
      offset += sizeof(this->recognition_id);
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.base = 0;
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->confidence = u_confidence.real;
      offset += sizeof(this->confidence);
     return offset;
    }

    const char * getType(){ return RECOGNIZE; };
    const char * getMD5(){ return "acd70f9d7ec7618a11f1502c493b2e64"; };

  };

  class Recognize {
    public:
    typedef RecognizeRequest Request;
    typedef RecognizeResponse Response;
  };

}
#endif
