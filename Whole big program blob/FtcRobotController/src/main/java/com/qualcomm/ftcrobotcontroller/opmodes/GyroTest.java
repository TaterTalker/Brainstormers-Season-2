package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by ethan on 11/30/2015.
 */
public class GyroTest extends OpMode {

 GyroSensor gyroSensor;

    @Override
    public void init() {

       gyroSensor = hardwareMap.gyroSensor.get("G1");
        gyroSensor.calibrate();
    }

    @Override
    public void start() {

        super.start();
    }

    @Override
    public void loop() {

        telemetry.addData("sav1", "SensorXYZ: " + gyroSensor.rawX()+" "+gyroSensor.rawY()+" "+gyroSensor.rawZ());
        telemetry.addData("heading: ", "" + gyroSensor.getHeading());

    }
}
