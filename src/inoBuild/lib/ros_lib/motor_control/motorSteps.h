#ifndef _ROS_motor_control_motorSteps_h
#define _ROS_motor_control_motorSteps_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace motor_control
{

  class motorSteps : public ros::Msg
  {
    public:
      typedef int32_t _baseStep_type;
      _baseStep_type baseStep;
      typedef int32_t _mainStep_type;
      _mainStep_type mainStep;
      typedef int32_t _secStep_type;
      _secStep_type secStep;
      typedef int32_t _toolStep_type;
      _toolStep_type toolStep;

    motorSteps():
      baseStep(0),
      mainStep(0),
      secStep(0),
      toolStep(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_baseStep;
      u_baseStep.real = this->baseStep;
      *(outbuffer + offset + 0) = (u_baseStep.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_baseStep.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_baseStep.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_baseStep.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->baseStep);
      union {
        int32_t real;
        uint32_t base;
      } u_mainStep;
      u_mainStep.real = this->mainStep;
      *(outbuffer + offset + 0) = (u_mainStep.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mainStep.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mainStep.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mainStep.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mainStep);
      union {
        int32_t real;
        uint32_t base;
      } u_secStep;
      u_secStep.real = this->secStep;
      *(outbuffer + offset + 0) = (u_secStep.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_secStep.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_secStep.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_secStep.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->secStep);
      union {
        int32_t real;
        uint32_t base;
      } u_toolStep;
      u_toolStep.real = this->toolStep;
      *(outbuffer + offset + 0) = (u_toolStep.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_toolStep.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_toolStep.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_toolStep.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->toolStep);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_baseStep;
      u_baseStep.base = 0;
      u_baseStep.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_baseStep.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_baseStep.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_baseStep.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->baseStep = u_baseStep.real;
      offset += sizeof(this->baseStep);
      union {
        int32_t real;
        uint32_t base;
      } u_mainStep;
      u_mainStep.base = 0;
      u_mainStep.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mainStep.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mainStep.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mainStep.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mainStep = u_mainStep.real;
      offset += sizeof(this->mainStep);
      union {
        int32_t real;
        uint32_t base;
      } u_secStep;
      u_secStep.base = 0;
      u_secStep.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_secStep.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_secStep.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_secStep.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->secStep = u_secStep.real;
      offset += sizeof(this->secStep);
      union {
        int32_t real;
        uint32_t base;
      } u_toolStep;
      u_toolStep.base = 0;
      u_toolStep.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_toolStep.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_toolStep.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_toolStep.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->toolStep = u_toolStep.real;
      offset += sizeof(this->toolStep);
     return offset;
    }

    const char * getType(){ return "motor_control/motorSteps"; };
    const char * getMD5(){ return "f498a772762c69a12fb90c6312de6ba9"; };

  };

}
#endif
