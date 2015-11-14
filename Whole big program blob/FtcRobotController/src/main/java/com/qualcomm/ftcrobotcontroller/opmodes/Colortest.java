package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by ethan on 11/14/2015.
 */
public class Colortest extends OpMode {

    ColorSensor colorSensor1;
    ColorSensor colorSensor2;

    @Override
    public void init() {

        colorSensor1 = hardwareMap.colorSensor.get("cs1");
        colorSensor2 = hardwareMap.colorSensor.get("cs2");
        colorSensor1.enableLed(true);
        colorSensor2.enableLed(true);

    }


    @Override
    public void loop() {
        colorSensor1.enableLed(true);
        colorSensor2.enableLed(true);

        telemetry.addData("cs1" , "Colorsensor 1:" +colorSensor1.red()+" "+colorSensor1.green() + " " + colorSensor1.blue());

        telemetry.addData("cs2", "Colorsensor 2:" +colorSensor2.red()+" "+colorSensor2.green()+" "+colorSensor2.blue());

    }
}
