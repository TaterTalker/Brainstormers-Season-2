package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes.CameraOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by August on 2/7/2016.
 */
public class rampDetection extends CameraOp{
    @Override
    public void runOpMode() throws InterruptedException {
        startCam();
        waitForStart();
        while (true){
            telemetry.addData("left ", leftRed() + " right: " + rightRed());
        }
    }
}
