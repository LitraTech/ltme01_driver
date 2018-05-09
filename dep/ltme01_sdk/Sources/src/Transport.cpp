#include "ltme01_sdk/Transport.h"

#include <typeinfo>

#include "ltme01_sdk/usb/UsbLocation.h"
#include "ltme01_sdk/usb/UsbTransport.h"

ltme01_sdk::Transport* ltme01_sdk::Transport::createInstance(const Location& location)
{
  if (typeid(location) == typeid(UsbLocation))
    return new UsbTransport(dynamic_cast<const UsbLocation&>(location));
  else
    return NULL;
}
