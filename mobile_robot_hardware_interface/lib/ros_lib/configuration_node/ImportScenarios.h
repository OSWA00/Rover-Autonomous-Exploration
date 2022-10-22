#ifndef _ROS_SERVICE_ImportScenarios_h
#define _ROS_SERVICE_ImportScenarios_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

static const char IMPORTSCENARIOS[] = "configuration_node/ImportScenarios";

  class ImportScenariosRequest : public ros::Msg
  {
    public:
      typedef const char* _file_type;
      _file_type file;

    ImportScenariosRequest():
      file("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_file = strlen(this->file);
      varToArr(outbuffer + offset, length_file);
      offset += 4;
      memcpy(outbuffer + offset, this->file, length_file);
      offset += length_file;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_file;
      arrToVar(length_file, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_file; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_file-1]=0;
      this->file = (char *)(inbuffer + offset-1);
      offset += length_file;
     return offset;
    }

    const char * getType(){ return IMPORTSCENARIOS; };
    const char * getMD5(){ return "e5aa404286053ca7f6cc32f23f37b85d"; };

  };

  class ImportScenariosResponse : public ros::Msg
  {
    public:
      typedef const char* _res_type;
      _res_type res;

    ImportScenariosResponse():
      res("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_res = strlen(this->res);
      varToArr(outbuffer + offset, length_res);
      offset += 4;
      memcpy(outbuffer + offset, this->res, length_res);
      offset += length_res;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_res;
      arrToVar(length_res, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_res; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_res-1]=0;
      this->res = (char *)(inbuffer + offset-1);
      offset += length_res;
     return offset;
    }

    const char * getType(){ return IMPORTSCENARIOS; };
    const char * getMD5(){ return "53af918a2a4a2a182c184142fff49b0c"; };

  };

  class ImportScenarios {
    public:
    typedef ImportScenariosRequest Request;
    typedef ImportScenariosResponse Response;
  };

}
#endif
