#ifndef _ROS_SERVICE_GetRobots_h
#define _ROS_SERVICE_GetRobots_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

static const char GETROBOTS[] = "configuration_node/GetRobots";

  class GetRobotsRequest : public ros::Msg
  {
    public:

    GetRobotsRequest()
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

    const char * getType(){ return GETROBOTS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetRobotsResponse : public ros::Msg
  {
    public:
      uint32_t robots_length;
      typedef char* _robots_type;
      _robots_type st_robots;
      _robots_type * robots;

    GetRobotsResponse():
      robots_length(0), robots(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->robots_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->robots_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->robots_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->robots_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->robots_length);
      for( uint32_t i = 0; i < robots_length; i++){
      uint32_t length_robotsi = strlen(this->robots[i]);
      varToArr(outbuffer + offset, length_robotsi);
      offset += 4;
      memcpy(outbuffer + offset, this->robots[i], length_robotsi);
      offset += length_robotsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t robots_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      robots_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      robots_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      robots_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->robots_length);
      if(robots_lengthT > robots_length)
        this->robots = (char**)realloc(this->robots, robots_lengthT * sizeof(char*));
      robots_length = robots_lengthT;
      for( uint32_t i = 0; i < robots_length; i++){
      uint32_t length_st_robots;
      arrToVar(length_st_robots, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_robots; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_robots-1]=0;
      this->st_robots = (char *)(inbuffer + offset-1);
      offset += length_st_robots;
        memcpy( &(this->robots[i]), &(this->st_robots), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return GETROBOTS; };
    const char * getMD5(){ return "2454c7482f287119cbb3968ac2fab774"; };

  };

  class GetRobots {
    public:
    typedef GetRobotsRequest Request;
    typedef GetRobotsResponse Response;
  };

}
#endif
