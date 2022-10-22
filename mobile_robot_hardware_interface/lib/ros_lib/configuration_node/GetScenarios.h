#ifndef _ROS_SERVICE_GetScenarios_h
#define _ROS_SERVICE_GetScenarios_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "configuration_node/Scenario.h"

namespace configuration_node
{

static const char GETSCENARIOS[] = "configuration_node/GetScenarios";

  class GetScenariosRequest : public ros::Msg
  {
    public:

    GetScenariosRequest()
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

    const char * getType(){ return GETSCENARIOS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetScenariosResponse : public ros::Msg
  {
    public:
      uint32_t scenarios_length;
      typedef configuration_node::Scenario _scenarios_type;
      _scenarios_type st_scenarios;
      _scenarios_type * scenarios;

    GetScenariosResponse():
      scenarios_length(0), scenarios(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->scenarios_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->scenarios_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->scenarios_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->scenarios_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->scenarios_length);
      for( uint32_t i = 0; i < scenarios_length; i++){
      offset += this->scenarios[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t scenarios_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      scenarios_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      scenarios_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      scenarios_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->scenarios_length);
      if(scenarios_lengthT > scenarios_length)
        this->scenarios = (configuration_node::Scenario*)realloc(this->scenarios, scenarios_lengthT * sizeof(configuration_node::Scenario));
      scenarios_length = scenarios_lengthT;
      for( uint32_t i = 0; i < scenarios_length; i++){
      offset += this->st_scenarios.deserialize(inbuffer + offset);
        memcpy( &(this->scenarios[i]), &(this->st_scenarios), sizeof(configuration_node::Scenario));
      }
     return offset;
    }

    const char * getType(){ return GETSCENARIOS; };
    const char * getMD5(){ return "60a07e8e4a151b4d14097c5fb19b52b4"; };

  };

  class GetScenarios {
    public:
    typedef GetScenariosRequest Request;
    typedef GetScenariosResponse Response;
  };

}
#endif
