#ifndef _ROS_SERVICE_FindRunningNodes_h
#define _ROS_SERVICE_FindRunningNodes_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

static const char FINDRUNNINGNODES[] = "configuration_node/FindRunningNodes";

  class FindRunningNodesRequest : public ros::Msg
  {
    public:
      typedef const char* _launchFilePath_type;
      _launchFilePath_type launchFilePath;

    FindRunningNodesRequest():
      launchFilePath("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_launchFilePath = strlen(this->launchFilePath);
      varToArr(outbuffer + offset, length_launchFilePath);
      offset += 4;
      memcpy(outbuffer + offset, this->launchFilePath, length_launchFilePath);
      offset += length_launchFilePath;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_launchFilePath;
      arrToVar(length_launchFilePath, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_launchFilePath; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_launchFilePath-1]=0;
      this->launchFilePath = (char *)(inbuffer + offset-1);
      offset += length_launchFilePath;
     return offset;
    }

    const char * getType(){ return FINDRUNNINGNODES; };
    const char * getMD5(){ return "fe746150bd5c9ab3386bcdb440e4cd99"; };

  };

  class FindRunningNodesResponse : public ros::Msg
  {
    public:
      uint32_t runningNodeList_length;
      typedef char* _runningNodeList_type;
      _runningNodeList_type st_runningNodeList;
      _runningNodeList_type * runningNodeList;

    FindRunningNodesResponse():
      runningNodeList_length(0), runningNodeList(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->runningNodeList_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->runningNodeList_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->runningNodeList_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->runningNodeList_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->runningNodeList_length);
      for( uint32_t i = 0; i < runningNodeList_length; i++){
      uint32_t length_runningNodeListi = strlen(this->runningNodeList[i]);
      varToArr(outbuffer + offset, length_runningNodeListi);
      offset += 4;
      memcpy(outbuffer + offset, this->runningNodeList[i], length_runningNodeListi);
      offset += length_runningNodeListi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t runningNodeList_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      runningNodeList_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      runningNodeList_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      runningNodeList_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->runningNodeList_length);
      if(runningNodeList_lengthT > runningNodeList_length)
        this->runningNodeList = (char**)realloc(this->runningNodeList, runningNodeList_lengthT * sizeof(char*));
      runningNodeList_length = runningNodeList_lengthT;
      for( uint32_t i = 0; i < runningNodeList_length; i++){
      uint32_t length_st_runningNodeList;
      arrToVar(length_st_runningNodeList, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_runningNodeList; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_runningNodeList-1]=0;
      this->st_runningNodeList = (char *)(inbuffer + offset-1);
      offset += length_st_runningNodeList;
        memcpy( &(this->runningNodeList[i]), &(this->st_runningNodeList), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return FINDRUNNINGNODES; };
    const char * getMD5(){ return "f4a14e15309df867c29fafbdbcedb4e5"; };

  };

  class FindRunningNodes {
    public:
    typedef FindRunningNodesRequest Request;
    typedef FindRunningNodesResponse Response;
  };

}
#endif
