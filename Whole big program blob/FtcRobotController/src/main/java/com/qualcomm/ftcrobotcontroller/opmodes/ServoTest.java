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
public class ServoTest extends OpMode {

    Servo linear;
    Servo quarter;

    @Override
    public void init() {
        linear = hardwareMap.servo.get("linear");
        quarter = hardwareMap.servo.get("qtr");
    }

    @Override
    public void stop() {

    }

    public void loop() {

        linear.setPosition(Math.abs(gamepad1.left_stick_x));
        quarter.setPosition(Math.abs(gamepad1.right_stick_x));

    }
}

