#ifndef _ROS_SERVICE_RemoveScenarios_h
#define _ROS_SERVICE_RemoveScenarios_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

static const char REMOVESCENARIOS[] = "configuration_node/RemoveScenarios";

  class RemoveScenariosRequest : public ros::Msg
  {
    public:
      uint32_t scenarioName_length;
      typedef char* _scenarioName_type;
      _scenarioName_type st_scenarioName;
      _scenarioName_type * scenarioName;

    RemoveScenariosRequest():
      scenarioName_length(0), scenarioName(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->scenarioName_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->scenarioName_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->scenarioName_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->scenarioName_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->scenarioName_length);
      for( uint32_t i = 0; i < scenarioName_length; i++){
      uint32_t length_scenarioNamei = strlen(this->scenarioName[i]);
      varToArr(outbuffer + offset, length_scenarioNamei);
      offset += 4;
      memcpy(outbuffer + offset, this->scenarioName[i], length_scenarioNamei);
      offset += length_scenarioNamei;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t scenarioName_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      scenarioName_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      scenarioName_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      scenarioName_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->scenarioName_length);
      if(scenarioName_lengthT > scenarioName_length)
        this->scenarioName = (char**)realloc(this->scenarioName, scenarioName_lengthT * sizeof(char*));
      scenarioName_length = scenarioName_lengthT;
      for( uint32_t i = 0; i < scenarioName_length; i++){
      uint32_t length_st_scenarioName;
      arrToVar(length_st_scenarioName, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_scenarioName; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_scenarioName-1]=0;
      this->st_scenarioName = (char *)(inbuffer + offset-1);
      offset += length_st_scenarioName;
        memcpy( &(this->scenarioName[i]), &(this->st_scenarioName), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return REMOVESCENARIOS; };
    const char * getMD5(){ return "9538f479b528e114501c4a515983365c"; };

  };

  class RemoveScenariosResponse : public ros::Msg
  {
    public:
      typedef bool _res_type;
      _res_type res;

    RemoveScenariosResponse():
      res(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_res;
      u_res.real = this->res;
      *(outbuffer + offset + 0) = (u_res.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->res);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_res;
      u_res.base = 0;
      u_res.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->res = u_res.real;
      offset += sizeof(this->res);
     return offset;
    }

    const char * getType(){ return REMOVESCENARIOS; };
    const char * getMD5(){ return "e27848a10f8e7e4030443887dfea101b"; };

  };

  class RemoveScenarios {
    public:
    typedef RemoveScenariosRequest Request;
    typedef RemoveScenariosResponse Response;
  };

}
#endif
