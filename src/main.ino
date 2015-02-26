#include "TCS34725.h"
#include "SoftI2CMaster.h"

#define ERROR_MESSAGE(err) Serial.println(err)

// pin definitions
int pinAssaySDA = D3;
int pinAssaySCL = D4;
int pinControlSDA = D5;
int pinControlSCL = D6;

TCS34725 tcsAssay;
TCS34725 tcsControl;


void init_sensor(TCS34725 *sensor, int sdaPin, int sclPin) {
    *sensor = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X, sdaPin, sclPin);

    if (sensor->begin()) {
        sensor->enable();
    }
    else {
        ERROR_MESSAGE("Sensor not found");
    }

    delay(1000); // let sensor "warm up"
}

void read_sensor(TCS34725 *sensor, int reading_number) {
    uint16_t clear, red, green, blue;

    sensor->getRawData(&red, &green, &blue, &clear);

    Serial.print("Reading number: ");
    Serial.print(reading_number);
    Serial.print(", Clear: ");
    Serial.print(clear);
    Serial.print(", Red: ");
    Serial.print(red);
    Serial.print(", Green: ");
    Serial.print(green);
    Serial.print(", Blue: ");
    Serial.println(blue);
}

void collect_sensor_readings() {
    for (int i = 0; i < 10; i += 1) { // take 10 sensor readings
        read_sensor(&tcsAssay, i);
        read_sensor(&tcsControl, i);
        delay(1000);
    }
}

//
//
//  SETUP
//
//

void setup() {
    pinMode(pinAssaySDA, OUTPUT); // setup sensor pins
    pinMode(pinAssaySCL, OUTPUT);
    pinMode(pinControlSDA, OUTPUT);
    pinMode(pinControlSCL, OUTPUT);

    Serial.begin(9600);
}

//
//
//  LOOP
//
//

void loop(){
    init_sensor(&tcsAssay, pinAssaySDA, pinAssaySCL);
    init_sensor(&tcsControl, pinControlSDA, pinControlSCL);

    collect_sensor_readings();

    tcsAssay.disable();
    tcsControl.disable();

    delay(10000); // wait 10 seconds until taking next reading

}
