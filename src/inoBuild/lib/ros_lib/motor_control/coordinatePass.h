#ifndef _ROS_SERVICE_coordinatePass_h
#define _ROS_SERVICE_coordinatePass_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace motor_control
{

static const char COORDINATEPASS[] = "motor_control/coordinatePass";

  class coordinatePassRequest : public ros::Msg
  {
    public:
      typedef bool _request_type;
      _request_type request;

    coordinatePassRequest():
      request(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_request;
      u_request.real = this->request;
      *(outbuffer + offset + 0) = (u_request.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->request);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_request;
      u_request.base = 0;
      u_request.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->request = u_request.real;
      offset += sizeof(this->request);
     return offset;
    }

    const char * getType(){ return COORDINATEPASS; };
    const char * getMD5(){ return "6f7e5ad6ab0ddf42c5727a195315a470"; };

  };

  class coordinatePassResponse : public ros::Msg
  {
    public:
      typedef float _xCoordinate_type;
      _xCoordinate_type xCoordinate;
      typedef float _yCoordinate_type;
      _yCoordinate_type yCoordinate;
      typedef float _zCoordinate_type;
      _zCoordinate_type zCoordinate;

    coordinatePassResponse():
      xCoordinate(0),
      yCoordinate(0),
      zCoordinate(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_xCoordinate;
      u_xCoordinate.real = this->xCoordinate;
      *(outbuffer + offset + 0) = (u_xCoordinate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xCoordinate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xCoordinate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xCoordinate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xCoordinate);
      union {
        float real;
        uint32_t base;
      } u_yCoordinate;
      u_yCoordinate.real = this->yCoordinate;
      *(outbuffer + offset + 0) = (u_yCoordinate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yCoordinate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yCoordinate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yCoordinate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yCoordinate);
      union {
        float real;
        uint32_t base;
      } u_zCoordinate;
      u_zCoordinate.real = this->zCoordinate;
      *(outbuffer + offset + 0) = (u_zCoordinate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zCoordinate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zCoordinate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zCoordinate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zCoordinate);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_xCoordinate;
      u_xCoordinate.base = 0;
      u_xCoordinate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xCoordinate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xCoordinate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xCoordinate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xCoordinate = u_xCoordinate.real;
      offset += sizeof(this->xCoordinate);
      union {
        float real;
        uint32_t base;
      } u_yCoordinate;
      u_yCoordinate.base = 0;
      u_yCoordinate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yCoordinate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yCoordinate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yCoordinate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yCoordinate = u_yCoordinate.real;
      offset += sizeof(this->yCoordinate);
      union {
        float real;
        uint32_t base;
      } u_zCoordinate;
      u_zCoordinate.base = 0;
      u_zCoordinate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zCoordinate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zCoordinate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zCoordinate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zCoordinate = u_zCoordinate.real;
      offset += sizeof(this->zCoordinate);
     return offset;
    }

    const char * getType(){ return COORDINATEPASS; };
    const char * getMD5(){ return "3e92147420f853642e8784f7a7bce474"; };

  };

  class coordinatePass {
    public:
    typedef coordinatePassRequest Request;
    typedef coordinatePassResponse Response;
  };

}
#endif
