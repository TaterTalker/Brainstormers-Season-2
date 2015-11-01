package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class TeleOp extends OpMode {
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor FR;
    DcMotor collector;
    DcMotor extendor1;
    DcMotor extendor2;
    DcMotor climber;
    TouchSensor TOUCHSENSOR1;
    TouchSensor TOUCHSENSOR2;
    Servo sideArmL;
    Servo sideArmR;
    Servo climberDumper;
    Servo debDumper;
    float YPower, XPower, rotPower;
    int direction = 1;
    int directionOld = 1;

    /**
     * Constructor
     */
    public TeleOp() {

    }

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {

        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        TOUCHSENSOR1 = hardwareMap.touchSensor.get("t1");
        TOUCHSENSOR2= hardwareMap.touchSensor.get("t2");
        BL = hardwareMap.dcMotor.get("BL");
        sideArmL = hardwareMap.servo.get("sideArmL");
        sideArmR = hardwareMap.servo.get("sideArmR");
        climberDumper = hardwareMap.servo.get("climberdumper");
        debDumper = hardwareMap.servo.get("debDumper");
        collector = hardwareMap.dcMotor.get("colmot");
        extendor1 = hardwareMap.dcMotor.get("ext1");
        extendor2 = hardwareMap.dcMotor.get("ext2");
        climber = hardwareMap.dcMotor.get("lock");
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        if (gamepad1.x && direction == directionOld) {
            direction *= -1;
        }
        directionOld = direction;

        drive();
        atatchmentControl();
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Forwards power", "" + String.format("%.2f", YPower));
        telemetry.addData("Left/Right Power", "" + String.format("%.2f", XPower));
        telemetry.addData("Rotation power", "" + String.format("%.2f", rotPower));
        telemetry.addData("Driving Direction", "" + String.format("%s", direction));

    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     *
     *
     *
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

    private void drive() {
        float YVal = direction * gamepad1.left_stick_y;
        float XVal = direction * gamepad1.left_stick_x;
        float RotVal = -gamepad1.right_stick_x;
        // clip the right/left values so that the values never exceed +/- 1
        YPower = Range.clip(YVal, -1, 1);
        XPower = Range.clip(XVal, -1, 1);
        rotPower = Range.clip(RotVal, -1, 1);

        float FRpower = YPower + XPower - rotPower;
        float FLpower = -(YPower - XPower + rotPower);
        float BRpower = YPower - XPower - rotPower;
        float BLpower = -(YPower + XPower + rotPower);

        FRpower = Range.clip(FRpower, -1, 1);
        FLpower = Range.clip(FLpower, -1, 1);
        BRpower = Range.clip(BRpower, -1, 1);
        BLpower = Range.clip(BLpower, -1, 1);

        // write the values to the motors
        FR.setPower(FRpower);
        BR.setPower(FLpower);
        FL.setPower(BRpower);
        BL.setPower(BLpower);
    }


    private void atatchmentControl() {

        int collectorval;
        if (gamepad2.right_bumper) {
            collectorval = 1;
        } else if (gamepad2.left_bumper) {
            collectorval = -1;
        } else collectorval = 0;
        collector.setPower(collectorval);

        if (gamepad2.dpad_right) {
            debDumper.setPosition(0.3);
        } else if (gamepad2.dpad_left) {
            debDumper.setPosition(0.7);
        } else debDumper.setPosition(0.5);
        collector.setPower(collectorval);

        if(gamepad2.a){
            sideArmL.setPosition(0);
            sideArmR.setPosition(1);
        }
        else{
            sideArmL.setPosition(1);
            sideArmR.setPosition(0);
        }

        if (gamepad2.y){
            climberDumper.setPosition(0.5);
        }
        else{
            climberDumper.setPosition(0);
        }

        if (gamepad2.right_trigger!=0){

            extendor1.setPower(1);
            extendor2.setPower(-1);
        }
        else if (gamepad2.left_trigger!=0){
            if(TOUCHSENSOR2.isPressed())extendor1.setPower(0);
            else extendor1.setPower(-1);
            if(TOUCHSENSOR1.isPressed()) extendor2.setPower(0);
            else extendor2.setPower(1);
        }
        else {
            extendor1.setPower(0);
            extendor2.setPower(0);
        }

        if (gamepad1.right_bumper) {
            climber.setPower(-1);
        } else climber.setPower(0);
    }
}
