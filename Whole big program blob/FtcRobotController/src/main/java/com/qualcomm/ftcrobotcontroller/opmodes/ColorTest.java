package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by ethan on 12/6/2015.
 */
public class ColorTest extends OpMode {

  ColorSensor colorSensor;

    @Override
    public void init() {
        colorSensor = hardwareMap.colorSensor.get("cs1") ;
        colorSensor.enableLed(true);

    }

    @Override
    public void loop() {
        colorSensor.enableLed(true);
        telemetry.addData("Green ", colorSensor.green());
        telemetry.addData("Red " , colorSensor.red());
        telemetry.addData("BLue ", colorSensor.blue());
        telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.addData("argb", colorSensor.argb());
    }
    public void stop() {
        colorSensor.enableLed(false);
    }
}
