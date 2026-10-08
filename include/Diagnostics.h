#pragma once

#include <Arduino.h>

/*
    The C3 adapter uses GPIO20/GPIO21 for TB6612 direction control, so UART0
    must not be used. Its PlatformIO profile enables the C3's separate native
    USB CDC/JTAG controller, which exposes Serial through the USB connector
    without changing either motor pin.
*/
namespace diagnostics {
inline auto port() -> decltype(Serial)& {
    return Serial;
}
}  // namespace diagnostics
