#ifndef LOCATION_H
#define LOCATION_H

#include "ltme01_sdk/Common.h"

namespace ltme01_sdk
{

class LTME01_SDK_API Location
{
public:
  virtual ~Location() = 0;
};

inline Location::~Location() = default;

}

#endif
