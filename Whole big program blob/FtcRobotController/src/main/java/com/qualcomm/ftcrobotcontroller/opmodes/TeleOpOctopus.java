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
public class TeleOpOctopus extends OpMode {

    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    DcMotor octr;
    DcMotor octl;
    float YPower, XPower, rotPower;


    @Override
    public void init() {

        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
    }

    @Override
    public void loop() {
        drive();
    }

    @Override
    public void stop() {

    }

    private void drive() {
        float YVal = gamepad1.left_stick_x;
        float XVal = gamepad1.left_stick_y;
        float RotVal = -gamepad1.right_stick_x;

        // clip the right/left values so that the values never exceed +/- 1
        YPower = Range.clip(YVal, -1, 1);
        XPower = Range.clip(XVal, -1, 1);
        rotPower = Range.clip(RotVal, -1, 1);

        float FRpower = -YPower - XPower + rotPower;
        float BRpower = -YPower + XPower + rotPower;
        float FLpower = YPower - XPower + rotPower;
        float BLpower = YPower + XPower + rotPower;

        FRpower = Range.clip(FRpower, -1, 1);
        FLpower = Range.clip(FLpower, -1, 1);
        BRpower = Range.clip(BRpower, -1, 1);
        BLpower = Range.clip(BLpower, -1, 1);

        // write the values to the motors
        fr.setPower(FRpower);
        octr.setPower(-FRpower);
        br.setPower(FLpower);
        fl.setPower(BRpower);
        octl.setPower(-BRpower);
        bl.setPower(BLpower);
    }
}

