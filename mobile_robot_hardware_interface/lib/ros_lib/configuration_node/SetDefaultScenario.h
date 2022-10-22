#ifndef _ROS_SERVICE_SetDefaultScenario_h
#define _ROS_SERVICE_SetDefaultScenario_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

static const char SETDEFAULTSCENARIO[] = "configuration_node/SetDefaultScenario";

  class SetDefaultScenarioRequest : public ros::Msg
  {
    public:
      typedef const char* _scenarioName_type;
      _scenarioName_type scenarioName;

    SetDefaultScenarioRequest():
      scenarioName("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_scenarioName = strlen(this->scenarioName);
      varToArr(outbuffer + offset, length_scenarioName);
      offset += 4;
      memcpy(outbuffer + offset, this->scenarioName, length_scenarioName);
      offset += length_scenarioName;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_scenarioName;
      arrToVar(length_scenarioName, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_scenarioName; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_scenarioName-1]=0;
      this->scenarioName = (char *)(inbuffer + offset-1);
      offset += length_scenarioName;
     return offset;
    }

    const char * getType(){ return SETDEFAULTSCENARIO; };
    const char * getMD5(){ return "01bfa348e12a2a9d51d269d400453e2e"; };

  };

  class SetDefaultScenarioResponse : public ros::Msg
  {
    public:
      typedef bool _res_type;
      _res_type res;

    SetDefaultScenarioResponse():
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

    const char * getType(){ return SETDEFAULTSCENARIO; };
    const char * getMD5(){ return "e27848a10f8e7e4030443887dfea101b"; };

  };

  class SetDefaultScenario {
    public:
    typedef SetDefaultScenarioRequest Request;
    typedef SetDefaultScenarioResponse Response;
  };

}
#endif
