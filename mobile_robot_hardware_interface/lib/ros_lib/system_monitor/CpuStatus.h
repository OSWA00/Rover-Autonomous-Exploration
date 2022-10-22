#ifndef _ROS_system_monitor_CpuStatus_h
#define _ROS_system_monitor_CpuStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace system_monitor
{

  class CpuStatus : public ros::Msg
  {
    public:
      typedef const char* _cpu_model_name_type;
      _cpu_model_name_type cpu_model_name;
      uint32_t cpu_utilizations_length;
      typedef float _cpu_utilizations_type;
      _cpu_utilizations_type st_cpu_utilizations;
      _cpu_utilizations_type * cpu_utilizations;
      uint32_t cpu_frequencies_length;
      typedef char* _cpu_frequencies_type;
      _cpu_frequencies_type st_cpu_frequencies;
      _cpu_frequencies_type * cpu_frequencies;
      typedef float _cpu_temperature_type;
      _cpu_temperature_type cpu_temperature;
      typedef float _mem_usage_percentage_type;
      _mem_usage_percentage_type mem_usage_percentage;

    CpuStatus():
      cpu_model_name(""),
      cpu_utilizations_length(0), cpu_utilizations(NULL),
      cpu_frequencies_length(0), cpu_frequencies(NULL),
      cpu_temperature(0),
      mem_usage_percentage(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_cpu_model_name = strlen(this->cpu_model_name);
      varToArr(outbuffer + offset, length_cpu_model_name);
      offset += 4;
      memcpy(outbuffer + offset, this->cpu_model_name, length_cpu_model_name);
      offset += length_cpu_model_name;
      *(outbuffer + offset + 0) = (this->cpu_utilizations_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cpu_utilizations_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cpu_utilizations_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cpu_utilizations_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_utilizations_length);
      for( uint32_t i = 0; i < cpu_utilizations_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cpu_utilizationsi;
      u_cpu_utilizationsi.real = this->cpu_utilizations[i];
      *(outbuffer + offset + 0) = (u_cpu_utilizationsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_utilizationsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_utilizationsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_utilizationsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_utilizations[i]);
      }
      *(outbuffer + offset + 0) = (this->cpu_frequencies_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cpu_frequencies_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cpu_frequencies_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cpu_frequencies_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_frequencies_length);
      for( uint32_t i = 0; i < cpu_frequencies_length; i++){
      uint32_t length_cpu_frequenciesi = strlen(this->cpu_frequencies[i]);
      varToArr(outbuffer + offset, length_cpu_frequenciesi);
      offset += 4;
      memcpy(outbuffer + offset, this->cpu_frequencies[i], length_cpu_frequenciesi);
      offset += length_cpu_frequenciesi;
      }
      union {
        float real;
        uint32_t base;
      } u_cpu_temperature;
      u_cpu_temperature.real = this->cpu_temperature;
      *(outbuffer + offset + 0) = (u_cpu_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_temperature);
      union {
        float real;
        uint32_t base;
      } u_mem_usage_percentage;
      u_mem_usage_percentage.real = this->mem_usage_percentage;
      *(outbuffer + offset + 0) = (u_mem_usage_percentage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mem_usage_percentage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mem_usage_percentage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mem_usage_percentage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mem_usage_percentage);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_cpu_model_name;
      arrToVar(length_cpu_model_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_cpu_model_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_cpu_model_name-1]=0;
      this->cpu_model_name = (char *)(inbuffer + offset-1);
      offset += length_cpu_model_name;
      uint32_t cpu_utilizations_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cpu_utilizations_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cpu_utilizations_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cpu_utilizations_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cpu_utilizations_length);
      if(cpu_utilizations_lengthT > cpu_utilizations_length)
        this->cpu_utilizations = (float*)realloc(this->cpu_utilizations, cpu_utilizations_lengthT * sizeof(float));
      cpu_utilizations_length = cpu_utilizations_lengthT;
      for( uint32_t i = 0; i < cpu_utilizations_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cpu_utilizations;
      u_st_cpu_utilizations.base = 0;
      u_st_cpu_utilizations.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cpu_utilizations.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cpu_utilizations.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cpu_utilizations.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cpu_utilizations = u_st_cpu_utilizations.real;
      offset += sizeof(this->st_cpu_utilizations);
        memcpy( &(this->cpu_utilizations[i]), &(this->st_cpu_utilizations), sizeof(float));
      }
      uint32_t cpu_frequencies_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cpu_frequencies_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cpu_frequencies_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cpu_frequencies_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cpu_frequencies_length);
      if(cpu_frequencies_lengthT > cpu_frequencies_length)
        this->cpu_frequencies = (char**)realloc(this->cpu_frequencies, cpu_frequencies_lengthT * sizeof(char*));
      cpu_frequencies_length = cpu_frequencies_lengthT;
      for( uint32_t i = 0; i < cpu_frequencies_length; i++){
      uint32_t length_st_cpu_frequencies;
      arrToVar(length_st_cpu_frequencies, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_cpu_frequencies; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_cpu_frequencies-1]=0;
      this->st_cpu_frequencies = (char *)(inbuffer + offset-1);
      offset += length_st_cpu_frequencies;
        memcpy( &(this->cpu_frequencies[i]), &(this->st_cpu_frequencies), sizeof(char*));
      }
      union {
        float real;
        uint32_t base;
      } u_cpu_temperature;
      u_cpu_temperature.base = 0;
      u_cpu_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cpu_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cpu_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cpu_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cpu_temperature = u_cpu_temperature.real;
      offset += sizeof(this->cpu_temperature);
      union {
        float real;
        uint32_t base;
      } u_mem_usage_percentage;
      u_mem_usage_percentage.base = 0;
      u_mem_usage_percentage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mem_usage_percentage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mem_usage_percentage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mem_usage_percentage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mem_usage_percentage = u_mem_usage_percentage.real;
      offset += sizeof(this->mem_usage_percentage);
     return offset;
    }

    const char * getType(){ return "system_monitor/CpuStatus"; };
    const char * getMD5(){ return "68a80231c12cbc54dca20331e91cbcff"; };

  };

}
#endif