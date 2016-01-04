
package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * the Adafruit RGB Sensor.  It assumes that the I2C
 * cable for the sensor is connected to an I2C port on the
 * Core Device Interface Module.
 *
 * It also assuems that the LED pin of the sensor is connected
 * to the digital signal pin of a digital port on the
 * Core Device Interface Module.
 *
 * You can use the digital port to turn the sensor's onboard
 * LED on or off.
 *
 * The op mode assumes that the Core Device Interface Module
 * is configured with a name of "dim" and that the Adafruit color sensor
 * is configured as an I2C device with a name of "color".
 *
 * It also assumes that the LED pin of the RGB sensor
 * is connected to the signal pin of digital port #5 (zero indexed)
 * of the Core Device Interface Module.
 *
 * You can use the X button on either gamepad to turn the LED on and off.
 *
 */
public class ColorTest extends LinearOpMode {
    ColorSensor cs1;
    @Override
    public void runOpMode() throws InterruptedException {
        cs1 = hardwareMap.colorSensor.get("cs1");
        while(true){
            telemetry.addData(
                    "values", "\nR" + cs1.red() +
                    "\nB" + cs1.blue() +
                    "\nG" + cs1.green() +
                    "\nA" + cs1.alpha() //USE ALPHA FOR WHITE LINE DETECTION
            );

            waitOneFullHardwareCycle();
        }
    }
}