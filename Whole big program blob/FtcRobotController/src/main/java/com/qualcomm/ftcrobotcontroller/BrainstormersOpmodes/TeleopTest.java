package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ethan on 2/15/2016.
 */
public class TeleopTest extends OpMode {

    Servo beacon;
    float servoheight;

    @Override
    public void init() {
        beacon = hardwareMap.servo.get("beacon");
        servoheight = 0.5f;
    }

    @Override
    public void loop() {

        if(gamepad1.a){
            servoheight+= 0.01;
        }
        if(gamepad1.b){
            servoheight-= 0.01;
        }

        beacon.setPosition(servoheight);
    }
}
