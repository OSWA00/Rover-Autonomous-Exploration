#ifndef _ROS_SERVICE_GetNodesDetails_h
#define _ROS_SERVICE_GetNodesDetails_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "configuration_node/NodeDetails.h"

namespace configuration_node
{

static const char GETNODESDETAILS[] = "configuration_node/GetNodesDetails";

  class GetNodesDetailsRequest : public ros::Msg
  {
    public:

    GetNodesDetailsRequest()
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

    const char * getType(){ return GETNODESDETAILS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetNodesDetailsResponse : public ros::Msg
  {
    public:
      uint32_t nodes_length;
      typedef configuration_node::NodeDetails _nodes_type;
      _nodes_type st_nodes;
      _nodes_type * nodes;

    GetNodesDetailsResponse():
      nodes_length(0), nodes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      uint32_t nodes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nodes_length);
      if(nodes_lengthT > nodes_length)
        this->nodes = (configuration_node::NodeDetails*)realloc(this->nodes, nodes_lengthT * sizeof(configuration_node::NodeDetails));
      nodes_length = nodes_lengthT;
      for( uint32_t i = 0; i < nodes_length; i++){
      offset += this->st_nodes.deserialize(inbuffer + offset);
        memcpy( &(this->nodes[i]), &(this->st_nodes), sizeof(configuration_node::NodeDetails));
      }
     return offset;
    }

    const char * getType(){ return GETNODESDETAILS; };
    const char * getMD5(){ return "4133a0ba09d25b9dba23d02cf47d6fa4"; };

  };

  class GetNodesDetails {
    public:
    typedef GetNodesDetailsRequest Request;
    typedef GetNodesDetailsResponse Response;
  };

}
#endif
