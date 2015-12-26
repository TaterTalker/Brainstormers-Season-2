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
public class TeleOpOctopus2 extends OpMode {


    //drive
    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    DcMotor octr;
    DcMotor octl;
    float YPower1, XPower1, rotPower, YPower2, XPower2 ;

    //scoring
    DcMotor collect;
    Servo armAngle;
    DcMotor pullUp;
    DcMotor ext;
    Servo climberDumper;
    Servo sideArmL;
    Servo sideArmR;
    TouchSensor extStop;



    @Override
    public void init() {

        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");

        octl = hardwareMap.dcMotor.get("octl");
        octr = hardwareMap.dcMotor.get("octr");

        collect = hardwareMap.dcMotor.get("collect");
        armAngle = hardwareMap.servo.get("armAngle");
        pullUp = hardwareMap.dcMotor.get("pullUp");
        ext = hardwareMap.dcMotor.get("ext");
        climberDumper = hardwareMap.servo.get("climberDumper");
        sideArmL = hardwareMap.servo.get("sideArmL");
        sideArmR = hardwareMap.servo.get("sideArmR");
        extStop = hardwareMap.touchSensor.get("extStop");


    }

    @Override
    public void loop() {
        drive();
        attachments();
    }

    @Override
    public void stop() {

    }

    private void drive() {
        float YVal1 = gamepad1.left_stick_y;
        float RotVal = gamepad1.left_stick_x;

        // clip the right/left values so that the values never exceed +/- 1
        YPower1 = Range.clip(YVal1, -1, 1);
        rotPower = Range.clip(RotVal, -1, 1);

        float FRpower = -YPower1 + rotPower;
        float BRpower = -YPower1 + rotPower;
        float FLpower = YPower1 + rotPower;
        float BLpower = YPower1 + rotPower;

        FRpower = Range.clip(FRpower, -1, 1);
        FLpower = Range.clip(FLpower, -1, 1);
        BRpower = Range.clip(BRpower, -1, 1);
        BLpower = Range.clip(BLpower, -1, 1);

        // write the values to the motors
        fr.setPower(FRpower);
        br.setPower(FLpower);
        fl.setPower(BRpower);
        bl.setPower(BLpower);

    }

    public void attachments() {
        int collectorval;
        //collection in
        if (gamepad2.right_bumper) {
            collectorval = -1;
            //collection out
        }

        else if (gamepad2.left_bumper) {
            collectorval = 1;
            //resting
        }

        else collectorval = 0;

        collect.setPower(collectorval);

        if(gamepad2.b){
            sideArmR.setPosition(1);
        }
        else{
            sideArmR.setPosition(0);
        }

        if(gamepad2.x){
            sideArmL.setPosition(0);
        }
        else{
            sideArmL.setPosition(1);
        }

        if (gamepad2.y){
            climberDumper.setPosition(0);
        }
        else{
            climberDumper.setPosition(1);
        }

        if (gamepad2.right_trigger!=0){
            ext.setPower(-1);
            pullUp.setPower(1);
        }

        else if (gamepad2.left_trigger!=0) {
            if (extStop.isPressed()) {
                ext.setPower(0);
                pullUp.setPower(0);
            }
            else {
                ext.setPower(1);
                pullUp.setPower(-1);
            }
        }

        else {
            ext.setPower(0);
            pullUp.setPower(0);
        }

        float YVal2 = gamepad2.left_stick_y;

        YPower2 = Range.clip(YVal2, -1, 1);

        armAngle.setPosition(YPower2);


    }
}

