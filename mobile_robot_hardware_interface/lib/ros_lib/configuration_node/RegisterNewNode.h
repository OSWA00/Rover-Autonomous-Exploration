#ifndef _ROS_SERVICE_RegisterNewNode_h
#define _ROS_SERVICE_RegisterNewNode_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

static const char REGISTERNEWNODE[] = "configuration_node/RegisterNewNode";

  class RegisterNewNodeRequest : public ros::Msg
  {
    public:
      typedef const char* _nodeName_type;
      _nodeName_type nodeName;
      typedef const char* _packageName_type;
      _packageName_type packageName;
      uint32_t runningNodeList_length;
      typedef char* _runningNodeList_type;
      _runningNodeList_type st_runningNodeList;
      _runningNodeList_type * runningNodeList;
      typedef const char* _launchFileName_type;
      _launchFileName_type launchFileName;
      typedef const char* _launchParams_type;
      _launchParams_type launchParams;

    RegisterNewNodeRequest():
      nodeName(""),
      packageName(""),
      runningNodeList_length(0), runningNodeList(NULL),
      launchFileName(""),
      launchParams("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_nodeName = strlen(this->nodeName);
      varToArr(outbuffer + offset, length_nodeName);
      offset += 4;
      memcpy(outbuffer + offset, this->nodeName, length_nodeName);
      offset += length_nodeName;
      uint32_t length_packageName = strlen(this->packageName);
      varToArr(outbuffer + offset, length_packageName);
      offset += 4;
      memcpy(outbuffer + offset, this->packageName, length_packageName);
      offset += length_packageName;
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
      uint32_t length_launchFileName = strlen(this->launchFileName);
      varToArr(outbuffer + offset, length_launchFileName);
      offset += 4;
      memcpy(outbuffer + offset, this->launchFileName, length_launchFileName);
      offset += length_launchFileName;
      uint32_t length_launchParams = strlen(this->launchParams);
      varToArr(outbuffer + offset, length_launchParams);
      offset += 4;
      memcpy(outbuffer + offset, this->launchParams, length_launchParams);
      offset += length_launchParams;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_nodeName;
      arrToVar(length_nodeName, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_nodeName; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_nodeName-1]=0;
      this->nodeName = (char *)(inbuffer + offset-1);
      offset += length_nodeName;
      uint32_t length_packageName;
      arrToVar(length_packageName, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_packageName; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_packageName-1]=0;
      this->packageName = (char *)(inbuffer + offset-1);
      offset += length_packageName;
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
      uint32_t length_launchFileName;
      arrToVar(length_launchFileName, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_launchFileName; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_launchFileName-1]=0;
      this->launchFileName = (char *)(inbuffer + offset-1);
      offset += length_launchFileName;
      uint32_t length_launchParams;
      arrToVar(length_launchParams, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_launchParams; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_launchParams-1]=0;
      this->launchParams = (char *)(inbuffer + offset-1);
      offset += length_launchParams;
     return offset;
    }

    const char * getType(){ return REGISTERNEWNODE; };
    const char * getMD5(){ return "e3dbd023a2f21cba578ae6a43b685880"; };

  };

  class RegisterNewNodeResponse : public ros::Msg
  {
    public:
      typedef bool _res_type;
      _res_type res;

    RegisterNewNodeResponse():
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

    const char * getType(){ return REGISTERNEWNODE; };
    const char * getMD5(){ return "e27848a10f8e7e4030443887dfea101b"; };

  };

  class RegisterNewNode {
    public:
    typedef RegisterNewNodeRequest Request;
    typedef RegisterNewNodeResponse Response;
  };

}
#endif
