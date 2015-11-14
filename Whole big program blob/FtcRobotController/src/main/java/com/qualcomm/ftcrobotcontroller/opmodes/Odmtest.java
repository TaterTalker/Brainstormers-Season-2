package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by ethan and dan on 11/14/2015.
 */
public class Odmtest extends OpMode {

    OpticalDistanceSensor odmtry1;
    OpticalDistanceSensor odmtry2;

    @Override
    public void init() {

        odmtry1 = hardwareMap.opticalDistanceSensor.get("odm1");
        odmtry2 = hardwareMap.opticalDistanceSensor.get("odm2");
    }

    @Override
    public void start() {

        super.start();
    }

    @Override
    public void loop() {

        telemetry.addData("sav1", "Sensor1: " + odmtry1.getLightDetectedRaw());
        telemetry.addData("sav2", "Sensor2: " + odmtry2.getLightDetectedRaw());

    }
}
