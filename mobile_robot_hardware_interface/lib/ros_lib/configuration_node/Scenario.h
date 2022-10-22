#ifndef _ROS_configuration_node_Scenario_h
#define _ROS_configuration_node_Scenario_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "configuration_node/NodeLaunchParam.h"

namespace configuration_node
{

  class Scenario : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      uint32_t nodes_length;
      typedef char* _nodes_type;
      _nodes_type st_nodes;
      _nodes_type * nodes;
      typedef bool _isActive_type;
      _isActive_type isActive;
      typedef bool _isRunning_type;
      _isRunning_type isRunning;
      uint32_t launchparams_length;
      typedef configuration_node::NodeLaunchParam _launchparams_type;
      _launchparams_type st_launchparams;
      _launchparams_type * launchparams;

    Scenario():
      name(""),
      nodes_length(0), nodes(NULL),
      isActive(0),
      isRunning(0),
      launchparams_length(0), launchparams(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
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
      union {
        bool real;
        uint8_t base;
      } u_isActive;
      u_isActive.real = this->isActive;
      *(outbuffer + offset + 0) = (u_isActive.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isActive);
      union {
        bool real;
        uint8_t base;
      } u_isRunning;
      u_isRunning.real = this->isRunning;
      *(outbuffer + offset + 0) = (u_isRunning.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isRunning);
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
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
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
      union {
        bool real;
        uint8_t base;
      } u_isActive;
      u_isActive.base = 0;
      u_isActive.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isActive = u_isActive.real;
      offset += sizeof(this->isActive);
      union {
        bool real;
        uint8_t base;
      } u_isRunning;
      u_isRunning.base = 0;
      u_isRunning.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isRunning = u_isRunning.real;
      offset += sizeof(this->isRunning);
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

    const char * getType(){ return "configuration_node/Scenario"; };
    const char * getMD5(){ return "6fedb7963ed8d7b1950ea7a700161824"; };

  };

}
#endif