#ifndef _ROS_SERVICE_CsGetSavedNetworks_h
#define _ROS_SERVICE_CsGetSavedNetworks_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace network_manager
{

static const char CSGETSAVEDNETWORKS[] = "network_manager/CsGetSavedNetworks";

  class CsGetSavedNetworksRequest : public ros::Msg
  {
    public:

    CsGetSavedNetworksRequest()
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

    const char * getType(){ return CSGETSAVEDNETWORKS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class CsGetSavedNetworksResponse : public ros::Msg
  {
    public:
      uint32_t networks_length;
      typedef char* _networks_type;
      _networks_type st_networks;
      _networks_type * networks;

    CsGetSavedNetworksResponse():
      networks_length(0), networks(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->networks_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->networks_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->networks_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->networks_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->networks_length);
      for( uint32_t i = 0; i < networks_length; i++){
      uint32_t length_networksi = strlen(this->networks[i]);
      varToArr(outbuffer + offset, length_networksi);
      offset += 4;
      memcpy(outbuffer + offset, this->networks[i], length_networksi);
      offset += length_networksi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t networks_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      networks_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      networks_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      networks_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->networks_length);
      if(networks_lengthT > networks_length)
        this->networks = (char**)realloc(this->networks, networks_lengthT * sizeof(char*));
      networks_length = networks_lengthT;
      for( uint32_t i = 0; i < networks_length; i++){
      uint32_t length_st_networks;
      arrToVar(length_st_networks, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_networks; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_networks-1]=0;
      this->st_networks = (char *)(inbuffer + offset-1);
      offset += length_st_networks;
        memcpy( &(this->networks[i]), &(this->st_networks), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return CSGETSAVEDNETWORKS; };
    const char * getMD5(){ return "abbfe8f252815fdf6503c6956f3b5a59"; };

  };

  class CsGetSavedNetworks {
    public:
    typedef CsGetSavedNetworksRequest Request;
    typedef CsGetSavedNetworksResponse Response;
  };

}
#endif
