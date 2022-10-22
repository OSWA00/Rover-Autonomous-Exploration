#ifndef _ROS_configuration_node_NodesStatus_h
#define _ROS_configuration_node_NodesStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "configuration_node/PackageStatus.h"

namespace configuration_node
{

  class NodesStatus : public ros::Msg
  {
    public:
      uint32_t packages_length;
      typedef configuration_node::PackageStatus _packages_type;
      _packages_type st_packages;
      _packages_type * packages;

    NodesStatus():
      packages_length(0), packages(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->packages_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->packages_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->packages_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->packages_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->packages_length);
      for( uint32_t i = 0; i < packages_length; i++){
      offset += this->packages[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t packages_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      packages_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      packages_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      packages_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->packages_length);
      if(packages_lengthT > packages_length)
        this->packages = (configuration_node::PackageStatus*)realloc(this->packages, packages_lengthT * sizeof(configuration_node::PackageStatus));
      packages_length = packages_lengthT;
      for( uint32_t i = 0; i < packages_length; i++){
      offset += this->st_packages.deserialize(inbuffer + offset);
        memcpy( &(this->packages[i]), &(this->st_packages), sizeof(configuration_node::PackageStatus));
      }
     return offset;
    }

    const char * getType(){ return "configuration_node/NodesStatus"; };
    const char * getMD5(){ return "7aefd21147eeaf8fa16445978205f3e1"; };

  };

}
#endif