package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by dan on 11/14/15.
 */
public class ultrasonictest extends OpMode {

    UltrasonicSensor ultra1;
    UltrasonicSensor ultra2;

    @Override
    public void init() {

        ultra1 = hardwareMap.ultrasonicSensor.get("ultra1");
        ultra2 = hardwareMap.ultrasonicSensor.get("ultra2");

    }

    @Override
    public void start() {

        super.start();
    }

    @Override
    public void loop() {

        telemetry.addData("sav1", "Sensor1: " + ultra1.getUltrasonicLevel());
        telemetry.addData("sav2", "Sensor2: " + ultra2.getUltrasonicLevel());

    }

}
