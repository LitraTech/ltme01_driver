#ifndef LOCATION_H
#define LOCATION_H

#include "ltme01_sdk/Common.h"

#include <memory>
#include <string>

namespace ltme01_sdk
{

class LTME01_SDK_API Location
{
public:
  virtual ~Location() = default;

  virtual std::unique_ptr<Location> clone() const = 0;
  virtual bool equals(const Location& other) const = 0;

  virtual std::string label() const = 0;
};

inline bool operator==(const Location& a, const Location& b)
{
  return a.equals(b);
}

}

#endif
