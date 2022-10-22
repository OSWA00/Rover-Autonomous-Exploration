#ifndef _ROS_SERVICE_ExportScenarios_h
#define _ROS_SERVICE_ExportScenarios_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

static const char EXPORTSCENARIOS[] = "configuration_node/ExportScenarios";

  class ExportScenariosRequest : public ros::Msg
  {
    public:
      uint32_t scenariosList_length;
      typedef char* _scenariosList_type;
      _scenariosList_type st_scenariosList;
      _scenariosList_type * scenariosList;

    ExportScenariosRequest():
      scenariosList_length(0), scenariosList(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->scenariosList_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->scenariosList_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->scenariosList_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->scenariosList_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->scenariosList_length);
      for( uint32_t i = 0; i < scenariosList_length; i++){
      uint32_t length_scenariosListi = strlen(this->scenariosList[i]);
      varToArr(outbuffer + offset, length_scenariosListi);
      offset += 4;
      memcpy(outbuffer + offset, this->scenariosList[i], length_scenariosListi);
      offset += length_scenariosListi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t scenariosList_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      scenariosList_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      scenariosList_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      scenariosList_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->scenariosList_length);
      if(scenariosList_lengthT > scenariosList_length)
        this->scenariosList = (char**)realloc(this->scenariosList, scenariosList_lengthT * sizeof(char*));
      scenariosList_length = scenariosList_lengthT;
      for( uint32_t i = 0; i < scenariosList_length; i++){
      uint32_t length_st_scenariosList;
      arrToVar(length_st_scenariosList, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_scenariosList; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_scenariosList-1]=0;
      this->st_scenariosList = (char *)(inbuffer + offset-1);
      offset += length_st_scenariosList;
        memcpy( &(this->scenariosList[i]), &(this->st_scenariosList), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return EXPORTSCENARIOS; };
    const char * getMD5(){ return "1f5ce236d4d8da3ec1866b62172fe42c"; };

  };

  class ExportScenariosResponse : public ros::Msg
  {
    public:
      typedef const char* _file_type;
      _file_type file;

    ExportScenariosResponse():
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

    const char * getType(){ return EXPORTSCENARIOS; };
    const char * getMD5(){ return "e5aa404286053ca7f6cc32f23f37b85d"; };

  };

  class ExportScenarios {
    public:
    typedef ExportScenariosRequest Request;
    typedef ExportScenariosResponse Response;
  };

}
#endif
