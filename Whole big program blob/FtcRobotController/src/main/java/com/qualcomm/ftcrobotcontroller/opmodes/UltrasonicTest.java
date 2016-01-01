package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by david on 11/30/2015.
 */
public class UltrasonicTest extends OpMode {

    AnalogInput ultra;

    @Override
    public void init() {

        ultra = hardwareMap.analogInput.get("ultra1");
    }

    @Override
    public void start() {

        super.start();
    }

    @Override
    public void loop() {
        ultra.getValue();
        telemetry.addData("ultra1", "Sensor1: " + ultra.getValue());

    }
}