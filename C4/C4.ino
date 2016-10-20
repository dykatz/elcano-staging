#include <serial-ring.h>

elcano::ParseState ps1, ps2;
elcano::SerialData dt1, dt2;
elcano::ParseStateError err;

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    Serial2.begin(9600);

    ps1.dt  = &dt1;
    ps1.dev = &Serial1;

    ps2.dt  = &dt2;
    ps2.dev = &Serial;
}

void loop() {
    err = ps1.update();
    if (err == elcano::ParseStateError::success) {
        if (dt1.kind == elcano::MsgType::goal) {
            dt1.write(&Serial);
        } else {
            dt1.write(&Serial2);
        }
    }

    err = ps2.update();
    if (err == elcano::ParseStateError::success) {
        dt2.write(&Serial2);
    }
}
