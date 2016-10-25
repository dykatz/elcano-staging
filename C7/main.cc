#include "serial-ring.hh"

/*
** main.cc
** By Dylan Katz
**
** The primary entry point for C7
*/

int main(int argc, char **argv) {
    serial::Serial     dev1("/dev/ttyACM0");
    serial::Serial     dev2("/dev/ttyACM1");
    elcano::ParseState ps1;
    elcano::SerialData dt1;
    
    ps1.dev = &dev1;
    ps1.dt  = &dt1;
    
    for (;;)
        ps1.update([&](elcano::SerialData *dt) { dt->write(dev2); });
}
