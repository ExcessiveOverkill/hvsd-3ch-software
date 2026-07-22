#include "device.h"

device* Device = nullptr;
#include "interrupt_catch.h"

int main(void) {
    device dev;
    Device = &dev;

    Device->init();
    Device->run();

    return 1;  // never reached
}
