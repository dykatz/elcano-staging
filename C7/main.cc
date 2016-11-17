#include <chrono>
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
    std::chrono::high_resolution_clock::time_point start, end;
    
    ps1.dev = &dev1;
    ps1.dt  = &dt1;
    start = std::chrono::high_resolution_clock::now();
    
    for (;;) {
        ps1.update([&](elcano::SerialData *dt) {
            dt->write(dev2);
            
            end = std::chrono::high_resolution_clock::now();
            std::cout << "Iteration took: " << (end - start) << " ms" << std::endl;
            start = end;
        });
    }
}
