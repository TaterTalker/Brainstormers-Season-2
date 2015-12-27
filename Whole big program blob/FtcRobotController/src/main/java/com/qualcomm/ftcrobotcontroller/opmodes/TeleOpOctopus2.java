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
    float YPower, rotPower, YPower2;

    //scoring
    DcMotor collect;
    Servo armAngle;
    DcMotor pullUp1;
    DcMotor pullUp2;
    DcMotor ext;
    Servo climberDumper;
    Servo sideArmL;
    Servo sideArmR;

    //Sensing
    TouchSensor extStop;



    @Override
    public void init() {

        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");

        collect = hardwareMap.dcMotor.get("collect");
        armAngle = hardwareMap.servo.get("armAngle");
        pullUp1 = hardwareMap.dcMotor.get("pullUp1");
        pullUp2 = hardwareMap.dcMotor.get("pullUp2");
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
        float YVal = gamepad1.left_stick_y;
        float RotVal = gamepad1.left_stick_x;

        // clip the right/left values so that the values never exceed +/- 1
        YPower = Range.clip(YVal, -1, 1);
        rotPower = Range.clip(RotVal, -1, 1);

        float FRpower = -YPower + rotPower;
        float BRpower = -YPower + rotPower;
        float FLpower = YPower + rotPower;
        float BLpower = YPower + rotPower;

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

        //collection in
        if (gamepad2.right_bumper) {
            collect.setPower(-1);
            //collection out
        }
        else if (gamepad2.left_bumper) {
            collect.setPower(1);
            //resting
        }
        else collect.setPower(0);

        //mountain climber release
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
        // climber dumper
        if (gamepad2.y){
            climberDumper.setPosition(0);
        }
        else{
            climberDumper.setPosition(1);
        }

        //arm
        if (gamepad2.right_trigger!=0){
            ext.setPower(-gamepad2.right_trigger);
            pullUp1.setPower(gamepad2.right_trigger);
            pullUp2.setPower(-gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger!=0) {
            if (extStop.isPressed()) {
                ext.setPower(0);
                pullUp1.setPower(0);
                pullUp2.setPower(0);
            }
            else {
                ext.setPower(gamepad1.right_trigger);
                pullUp1.setPower(-gamepad2.right_trigger);
                pullUp2.setPower(gamepad2.right_trigger);
            }
        }
        else {
            ext.setPower(0);
            pullUp1.setPower(0);
            pullUp2.setPower(0);
        }
        //arm angle

        float YVal2 = gamepad2.left_stick_y;

        YPower2 = Range.clip(YVal2, -1, 1);

        armAngle.setPosition(YPower2);


    }
}

