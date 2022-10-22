#ifndef _ROS_configuration_node_PackageStatus_h
#define _ROS_configuration_node_PackageStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "configuration_node/NodeStatus.h"

namespace configuration_node
{

  class PackageStatus : public ros::Msg
  {
    public:
      typedef const char* _packageName_type;
      _packageName_type packageName;
      uint32_t nodes_length;
      typedef configuration_node::NodeStatus _nodes_type;
      _nodes_type st_nodes;
      _nodes_type * nodes;

    PackageStatus():
      packageName(""),
      nodes_length(0), nodes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_packageName = strlen(this->packageName);
      varToArr(outbuffer + offset, length_packageName);
      offset += 4;
      memcpy(outbuffer + offset, this->packageName, length_packageName);
      offset += length_packageName;
      *(outbuffer + offset + 0) = (this->nodes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nodes_length);
      for( uint32_t i = 0; i < nodes_length; i++){
      offset += this->nodes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_packageName;
      arrToVar(length_packageName, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_packageName; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_packageName-1]=0;
      this->packageName = (char *)(inbuffer + offset-1);
      offset += length_packageName;
      uint32_t nodes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nodes_length);
      if(nodes_lengthT > nodes_length)
        this->nodes = (configuration_node::NodeStatus*)realloc(this->nodes, nodes_lengthT * sizeof(configuration_node::NodeStatus));
      nodes_length = nodes_lengthT;
      for( uint32_t i = 0; i < nodes_length; i++){
      offset += this->st_nodes.deserialize(inbuffer + offset);
        memcpy( &(this->nodes[i]), &(this->st_nodes), sizeof(configuration_node::NodeStatus));
      }
     return offset;
    }

    const char * getType(){ return "configuration_node/PackageStatus"; };
    const char * getMD5(){ return "def6979276bac8e1fdfea19037437ac2"; };

  };

}
#endif