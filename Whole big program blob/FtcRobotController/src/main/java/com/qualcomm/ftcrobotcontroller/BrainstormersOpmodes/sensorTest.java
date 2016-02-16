package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;


/**
 * prints all sensor values
 */
public class sensorTest extends LinearOpMode{
    //OpticalDistanceSensor odmtry1;
    GyroSensor gyroSensor;
    ColorSensor sensorRGB;
    UltrasonicSensor ultra1;
    Servo beacon;
    UltrasonicSensor ultra2;
    DeviceInterfaceModule cdim;
    static final int LED_CHANNEL = 5;


   public void runOpMode () throws InterruptedException{

       //odmtry1 = hardwareMap.opticalDistanceSensor.get("odm1");
       ultra1 = hardwareMap.ultrasonicSensor.get("ultraL");
       ultra2 = hardwareMap.ultrasonicSensor.get("ultraR");
       gyroSensor = hardwareMap.gyroSensor.get("G1");
       //cdim = hardwareMap.deviceInterfaceModule.get("dim");
       sensorRGB = hardwareMap.colorSensor.get("cs2");
       beacon = hardwareMap.servo.get("beacon");

       beacon.setPosition(0.6);
       //cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
       waitForStart();
       gyroSensor.calibrate();

while (true) {
    //telemetry.addData("ODM", "Sensor1: " + odmtry1.getLightDetectedRaw());
    telemetry.addData("Gyro", "SensorXYZ: " + gyroSensor.rawX() + " " + gyroSensor.rawY() + " " + gyroSensor.rawZ());
    telemetry.addData("Heading: ", gyroSensor.getHeading());
    telemetry.addData("Color Sensor:", "Alpha " + sensorRGB.alpha() + "Red " + sensorRGB.red() + "Green " + sensorRGB.green() + "Blue " + sensorRGB.blue());
    telemetry.addData("Ultrasonic", "SensorL: " + ultra1.getUltrasonicLevel() + "SensorR: " + ultra2.getUltrasonicLevel());
    telemetry.addData("Beaconservo", "Beacon: " + beacon.getPosition() );

    waitOneFullHardwareCycle();
}
   }

}
