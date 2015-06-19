# arduino-dw1000
A library that offers functionality to use Decawave's DW1000 chips/modules with Arduino/AVR
(see http://www.decawave.com/products/dwm1000-module).

Project state
-------------

**Progress:** 75% (to a first fully configurable, tested and stable release)

**Current milestone:** Tuning and testing (sender/receiver, ping-pong and two-way ranging)

**Subsequent milestone:** Frame filtering rules, nodes addressing and MAC data format

**General notice:** A more or less stable transmission of messages between two modules is possible at the momement. The code for device tuning currently has issues and is disabled. This may lead to a few percent increased rate of dropped messages (those not received at all or which did not pass the CRC check).

Contents
--------

 * [Project structure](../../wiki/Project-structure)
 * [Features and design intentions](../../wiki/Features)
 * [Testbed and Adapter board](../../wiki/Testbed-and-Adapter-board)
 * [Benchmarks](../../wiki/Benchmarks)

Usage
-----

General usage of the DW1000 library is as follows. API docs for the library will shortly follow.

```Arduino
#include <DW1000.h>
...
// init with interrupt line and optionally with reset line
DW1000.begin(irq_pin[, rst_pin]);
// select a specific chip via a chip select line
DW1000.select(cs_pin);
...
// open a device configuration sessions
DW1000.newConfiguration();
// configure specific aspects and/or choose defaults
DW1000.setDefaults();
DW1000.interruptOnSent(true);
DW1000.suppressFrameCheck(true);
// ... and other stuff - finally upload to the module.
DW1000.commitConfiguration();
...
// set some interrupt callback routines
DW1000.attachSentHandler(some_handler_function);
...
// open a new transmit session
DW1000.newTransmit();
// configure specific aspects and/or choose defaults
DW1000.setDefaults();
DW1000.setData(some_data);
[float futureTimestamp = ]DW1000.setDelay(100, DW1000.MILLISECONDS);
// ... and other stuff - finally start the transmission
DW1000.startTransmit();
...
// similar for a receiving session, like so ...
DW1000.newReceive();
DW1000.setDefaults();
// so we don't need to restart the receiver manually each time
DW1000.receivePermanently(true);
// ... and other stuff - finally start awaiting messages
DW1000.startReceive();
...
```


