#ifndef _ROS_SERVICE_CreateScenario_h
#define _ROS_SERVICE_CreateScenario_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "configuration_node/NodeLaunchParam.h"

namespace configuration_node
{

static const char CREATESCENARIO[] = "configuration_node/CreateScenario";

  class CreateScenarioRequest : public ros::Msg
  {
    public:
      typedef const char* _scenarioName_type;
      _scenarioName_type scenarioName;
      uint32_t nodes_length;
      typedef char* _nodes_type;
      _nodes_type st_nodes;
      _nodes_type * nodes;
      uint32_t launchparams_length;
      typedef configuration_node::NodeLaunchParam _launchparams_type;
      _launchparams_type st_launchparams;
      _launchparams_type * launchparams;

    CreateScenarioRequest():
      scenarioName(""),
      nodes_length(0), nodes(NULL),
      launchparams_length(0), launchparams(NULL)
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
      *(outbuffer + offset + 0) = (this->nodes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nodes_length);
      for( uint32_t i = 0; i < nodes_length; i++){
      uint32_t length_nodesi = strlen(this->nodes[i]);
      varToArr(outbuffer + offset, length_nodesi);
      offset += 4;
      memcpy(outbuffer + offset, this->nodes[i], length_nodesi);
      offset += length_nodesi;
      }
      *(outbuffer + offset + 0) = (this->launchparams_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->launchparams_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->launchparams_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->launchparams_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->launchparams_length);
      for( uint32_t i = 0; i < launchparams_length; i++){
      offset += this->launchparams[i].serialize(outbuffer + offset);
      }
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
      uint32_t nodes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nodes_length);
      if(nodes_lengthT > nodes_length)
        this->nodes = (char**)realloc(this->nodes, nodes_lengthT * sizeof(char*));
      nodes_length = nodes_lengthT;
      for( uint32_t i = 0; i < nodes_length; i++){
      uint32_t length_st_nodes;
      arrToVar(length_st_nodes, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_nodes; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_nodes-1]=0;
      this->st_nodes = (char *)(inbuffer + offset-1);
      offset += length_st_nodes;
        memcpy( &(this->nodes[i]), &(this->st_nodes), sizeof(char*));
      }
      uint32_t launchparams_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      launchparams_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      launchparams_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      launchparams_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->launchparams_length);
      if(launchparams_lengthT > launchparams_length)
        this->launchparams = (configuration_node::NodeLaunchParam*)realloc(this->launchparams, launchparams_lengthT * sizeof(configuration_node::NodeLaunchParam));
      launchparams_length = launchparams_lengthT;
      for( uint32_t i = 0; i < launchparams_length; i++){
      offset += this->st_launchparams.deserialize(inbuffer + offset);
        memcpy( &(this->launchparams[i]), &(this->st_launchparams), sizeof(configuration_node::NodeLaunchParam));
      }
     return offset;
    }

    const char * getType(){ return CREATESCENARIO; };
    const char * getMD5(){ return "3bdb05f62634dbe1582e4bd63e8ebd8f"; };

  };

  class CreateScenarioResponse : public ros::Msg
  {
    public:
      typedef bool _res_type;
      _res_type res;

    CreateScenarioResponse():
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

    const char * getType(){ return CREATESCENARIO; };
    const char * getMD5(){ return "e27848a10f8e7e4030443887dfea101b"; };

  };

  class CreateScenario {
    public:
    typedef CreateScenarioRequest Request;
    typedef CreateScenarioResponse Response;
  };

}
#endif
