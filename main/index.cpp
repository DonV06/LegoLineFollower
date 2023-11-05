#include <unistd.h>
#include <cstdio>

#include "ev3dev.h"

using namespace std;
using namespace ev3dev;

// CurrentReading

int currentReadL2, currentReadL1, currentReadR1, currentReadR2;

// Color and motors init (increment means further away)
color_sensor csL2 = color_sensor(INPUT_1);
color_sensor csL1 = color_sensor(INPUT_2);
color_sensor csR1 = color_sensor(INPUT_3);
color_sensor csR2 = color_sensor(INPUT_4);

medium_motor mm11 = medium_motor(OUTPUT_A);
medium_motor mm12 = medium_motor(OUTPUT_B);
medium_motor mm21 = medium_motor(OUTPUT_C);
medium_motor mm22 = medium_motor(OUTPUT_D);

double kP = 0.3, kI = 0.001, kD = 0.25;
// Sensor Limits
double error = 0;
double pidOutput = 0;
int minL1 = 100, minL2 = 100, minR1 = 100, minR2 = 100;
double errL2 = 0, errL1 = 0, errR1 = 0, errR2 = 0;
int maxL1 = 0, maxL2 = 0, maxR1 = 0, maxR2 = 0;
double previousError = 0, integral = 0, derivative = 0;
//Speed adjustments
int baseSpeed = 500;
double motorLeftSpeed = 0, motorRightSpeed = 0;

//Curve Acc false == left
int accStraight = 0, accCurve = 0;
bool curveDirection = false;


void setup() {
    int L2S = csL2.reflected_light_intensity();
    minL2 = std::min(minL2, L2S);
    maxL2 = std::max(maxL2, L2S);
    int L1S = csL1.reflected_light_intensity();
    minL1 = std::min(minL1, L1S);
    maxL1 = std::max(maxL1, L1S);
    int R1S = csR1.reflected_light_intensity();
    minR1 = std::min(minR1, R1S);
    maxR1 = std::max(maxR1, R1S);
    int R2S = csR2.reflected_light_intensity();
    minR2 = std::min(minR2, R2S);
    maxR2 = std::max(maxR2, R2S);

    currentReadL2 = 0;
    currentReadL1 = 0;
    currentReadR1 = 0;
    currentReadR2 = 0;

}

void PIDProgram() {
    currentReadL2 = csL2.reflected_light_intensity();
    currentReadL1 = csL1.reflected_light_intensity();
    currentReadR1 = csR1.reflected_light_intensity();
    currentReadR2 = csR2.reflected_light_intensity();

    errL2 = 100*(maxL2 - currentReadL2)/(maxL2 - minL2);
    errL1 = 10*(maxL1 - currentReadL1)/(maxL1 - minL1);
    errR1 = 10*(maxR1 - currentReadR1)/(maxR1 - minR1);
    errR2 = 100*(maxR2 - currentReadR2)/(maxR2 - minR2);
   

    if (maxL2 - currentReadL2 > maxR2 - currentReadR2 + 100) {
        curveDirection = false;
    } else if (maxL2 - currentReadL2 < maxR2 - currentReadR2 - 100) {
        curveDirection = true;
    }
    //If (leftOuterValue>maxL1-100) And (leftInnerValue>maxL2-100) And (rightInnerValue>maxR2-100) And (rightOuterValue>maxR1-100) Then
    if (currentReadL1 > maxL1 - 100 && currentReadL2 > maxL2 - 100 && currentReadR2 > maxR2 - 100 && currentReadR1 > maxR1 - 100) {
        if (curveDirection) {
            mm11.set_speed_sp(baseSpeed + accCurve);
            mm12.set_speed_sp(baseSpeed + accCurve);
            mm21.set_speed_sp(0);
            mm22.set_speed_sp(0);
            accCurve = accCurve +1 ;
        } else {
            mm11.set_speed_sp(0);
            mm12.set_speed_sp( 0 );
            mm21.set_speed_sp(-baseSpeed + accCurve);
            mm22.set_speed_sp(-baseSpeed + accCurve);
            accCurve = accCurve +1 ;

        }
        accStraight = 0;
        mm11.run_forever();
        mm12.run_forever();
        mm21.run_forever();
        mm22.run_forever();
        return;
    }else
    {
        accCurve = 0;
    }
    error = (errL2 + errL1) - (errR1 - errR2);
    integral = integral + error;
    derivative = error - previousError;
    pidOutput = (kP * error) + (kI * integral) + (kD * derivative);
    motorLeftSpeed = baseSpeed - pidOutput;
    motorRightSpeed = baseSpeed + pidOutput;
    if (motorLeftSpeed + accCurve < 900 ) {
        mm11.set_speed_sp(int(motorLeftSpeed + accCurve));
        mm12.set_speed_sp(int(motorLeftSpeed + accCurve);
    } else {
        mm11.set_speed_sp(900);
        mm12.set_speed_sp(900);

    }
    if (motorRightSpeed + accCurve < 900 ) {
        mm21.set_speed_sp(int(-motorRightSpeed + accCurve));
        mm22.set_speed_sp(int(-motorRightSpeed + accCurve));
    } else {
        mm21.set_speed_sp(-900);
        mm22.set_speed_sp(-900);
    }
    accStraight = accStraight + 1;
    previousError = error;
    mm11.run_forever();
    mm12.run_forever();
    mm21.run_forever();
    mm22.run_forever();

}


int main ()
{
    bool right = false;
    bool escape = false;
    bool left = false;

    while (!left) {
        left = button::left.pressed ();
    }

    while (!right) {
        right = button::right.pressed ();
        setup();
    }
    left = false;
    while (!left) {
        left = button::left.pressed ();
        PIDProgram();
    }



    while (escape == 0)
    {
        escape = button::back.pressed ();

        printf ("esc:%d\n",  escape);
    }
}
