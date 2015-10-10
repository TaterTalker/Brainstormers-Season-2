package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.ftcrobotcontroller.opmodes.mainDriving;

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
    DcMotor lock;
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


		/*
         * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */
		
		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *   
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        collector = hardwareMap.dcMotor.get("colmot");
        extendor1 = hardwareMap.dcMotor.get("ext1");
        extendor2 = hardwareMap.dcMotor.get("ext2");
        lock = hardwareMap.dcMotor.get("lock");
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
        if (gamepad2.dpad_up) {
            collectorval = 1;
        } else if (gamepad2.dpad_down) {
            collectorval = -1;
        } else collectorval = 0;
        collector.setPower(collectorval);

        if (gamepad1.right_bumper) {
            lock.setPower(0.2); 
        } else {
            lock.setPower(0);
        }
    }
}
