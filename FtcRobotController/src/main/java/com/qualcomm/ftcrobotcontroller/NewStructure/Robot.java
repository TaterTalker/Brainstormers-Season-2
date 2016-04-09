package com.qualcomm.ftcrobotcontroller.NewStructure;

import com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes.AdafruitIMUmethods;
import com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes.BackCameraController;
import com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes.FrontCameraController;
import com.qualcomm.ftcrobotcontroller.NewStructure.Parts.Arm;
import com.qualcomm.ftcrobotcontroller.NewStructure.Parts.Dumper;
import com.qualcomm.ftcrobotcontroller.NewStructure.Parts.SideArms;
import com.qualcomm.ftcrobotcontroller.NewStructure.Parts.WheelBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by ethan on 4/5/2016.
 */
public class Robot {
    //Drive


    int minpullup = 0;
    int side;
    double gyroOffset=0;

    //Controllers
    AdafruitIMUmethods adaFruitGyro;
    BackCameraController cameraController;
    public FrontCameraController frontCam;
   OpMode opMode;

    Arm arm;
    Dumper dumper;
    SideArms sideArms;
    WheelBase wheelBase;

    //Driving Motors

    DcMotor collector;
    ColorSensor colorSensor;
    UltrasonicSensor ultra1;
    UltrasonicSensor ultra2;

    //Servo
    Servo beaconR;
    Servo beaconL;
    Servo allClear;
    Servo climberDumper;


    Servo armHook;

    //Variables
    int turnDirection = 1;
    private boolean didEncodersReset = false;


    public Robot (int side , OpMode varopMode) {


        this.side = side;
        this.opMode = varopMode;


        arm = new Arm(opMode);
        dumper = new Dumper(side, opMode);
        sideArms = new SideArms( side,opMode);
        wheelBase = new WheelBase(opMode);

        //Sensors
        collector = opMode.hardwareMap.dcMotor.get("collect");
        climberDumper = opMode.hardwareMap.servo.get("climberDumper");
        beaconR = opMode.hardwareMap.servo.get("beacon right");
        beaconL = opMode.hardwareMap.servo.get("beacon left");


        armHook = opMode.hardwareMap.servo.get("armHook");
        colorSensor = opMode.hardwareMap.colorSensor.get("cs2");
        ultra1 = opMode.hardwareMap.ultrasonicSensor.get("ultraL");
        ultra2 = opMode.hardwareMap.ultrasonicSensor.get("ultraR");

        //Motors


        adaFruitGyro = new AdafruitIMUmethods(opMode);

        allClear = opMode.hardwareMap.servo.get("allClear");

    }









}
