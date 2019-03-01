#include "ltme01_sdk/Transport.h"

#include <typeinfo>

#include "ltme01_sdk/lan/LanLocation.h"
#include "ltme01_sdk/lan/LanTransport.h"

#include "ltme01_sdk/usb/UsbLocation.h"
#include "ltme01_sdk/usb/UsbTransport.h"

ltme01_sdk::Transport* ltme01_sdk::Transport::createInstance(const Location& location)
{
  if (typeid(location) == typeid(UsbLocation))
    return new UsbTransport(dynamic_cast<const UsbLocation&>(location));
  else if (typeid(location) == typeid(LanLocation))
    return new LanTransport(dynamic_cast<const LanLocation&>(location));
  else
    return NULL;
}
