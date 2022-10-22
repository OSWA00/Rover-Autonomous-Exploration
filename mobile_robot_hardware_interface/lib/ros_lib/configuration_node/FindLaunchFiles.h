#ifndef _ROS_SERVICE_FindLaunchFiles_h
#define _ROS_SERVICE_FindLaunchFiles_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

static const char FINDLAUNCHFILES[] = "configuration_node/FindLaunchFiles";

  class FindLaunchFilesRequest : public ros::Msg
  {
    public:
      typedef const char* _packageName_type;
      _packageName_type packageName;

    FindLaunchFilesRequest():
      packageName("")
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
     return offset;
    }

    const char * getType(){ return FINDLAUNCHFILES; };
    const char * getMD5(){ return "7cd9b5484c281924ee557bebc9b11dd5"; };

  };

  class FindLaunchFilesResponse : public ros::Msg
  {
    public:
      uint32_t launchFileList_length;
      typedef char* _launchFileList_type;
      _launchFileList_type st_launchFileList;
      _launchFileList_type * launchFileList;

    FindLaunchFilesResponse():
      launchFileList_length(0), launchFileList(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->launchFileList_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->launchFileList_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->launchFileList_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->launchFileList_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->launchFileList_length);
      for( uint32_t i = 0; i < launchFileList_length; i++){
      uint32_t length_launchFileListi = strlen(this->launchFileList[i]);
      varToArr(outbuffer + offset, length_launchFileListi);
      offset += 4;
      memcpy(outbuffer + offset, this->launchFileList[i], length_launchFileListi);
      offset += length_launchFileListi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t launchFileList_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      launchFileList_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      launchFileList_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      launchFileList_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->launchFileList_length);
      if(launchFileList_lengthT > launchFileList_length)
        this->launchFileList = (char**)realloc(this->launchFileList, launchFileList_lengthT * sizeof(char*));
      launchFileList_length = launchFileList_lengthT;
      for( uint32_t i = 0; i < launchFileList_length; i++){
      uint32_t length_st_launchFileList;
      arrToVar(length_st_launchFileList, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_launchFileList; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_launchFileList-1]=0;
      this->st_launchFileList = (char *)(inbuffer + offset-1);
      offset += length_st_launchFileList;
        memcpy( &(this->launchFileList[i]), &(this->st_launchFileList), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return FINDLAUNCHFILES; };
    const char * getMD5(){ return "eaa4de7dc6d0ac4648714d054969d026"; };

  };

  class FindLaunchFiles {
    public:
    typedef FindLaunchFilesRequest Request;
    typedef FindLaunchFilesResponse Response;
  };

}
#endif
