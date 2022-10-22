#ifndef _ROS_SERVICE_GetLaunchFilePath_h
#define _ROS_SERVICE_GetLaunchFilePath_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

static const char GETLAUNCHFILEPATH[] = "configuration_node/GetLaunchFilePath";

  class GetLaunchFilePathRequest : public ros::Msg
  {
    public:
      typedef const char* _packageName_type;
      _packageName_type packageName;
      typedef const char* _launchFileName_type;
      _launchFileName_type launchFileName;

    GetLaunchFilePathRequest():
      packageName(""),
      launchFileName("")
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
      uint32_t length_launchFileName = strlen(this->launchFileName);
      varToArr(outbuffer + offset, length_launchFileName);
      offset += 4;
      memcpy(outbuffer + offset, this->launchFileName, length_launchFileName);
      offset += length_launchFileName;
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
      uint32_t length_launchFileName;
      arrToVar(length_launchFileName, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_launchFileName; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_launchFileName-1]=0;
      this->launchFileName = (char *)(inbuffer + offset-1);
      offset += length_launchFileName;
     return offset;
    }

    const char * getType(){ return GETLAUNCHFILEPATH; };
    const char * getMD5(){ return "94b9b7508e10d2079fd6672ba349c2f6"; };

  };

  class GetLaunchFilePathResponse : public ros::Msg
  {
    public:
      typedef const char* _launchFilePath_type;
      _launchFilePath_type launchFilePath;

    GetLaunchFilePathResponse():
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

    const char * getType(){ return GETLAUNCHFILEPATH; };
    const char * getMD5(){ return "fe746150bd5c9ab3386bcdb440e4cd99"; };

  };

  class GetLaunchFilePath {
    public:
    typedef GetLaunchFilePathRequest Request;
    typedef GetLaunchFilePathResponse Response;
  };

}
#endif
