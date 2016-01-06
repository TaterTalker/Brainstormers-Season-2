package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class TeleOpOctopusTestPrgm extends OpMode {


    //drive
    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    float YPower, rotPower, YPower2;
    boolean armextended = true;
    int oldarm;

    //scoring
    DcMotor collect;
    Servo armAngle1;
    Servo armAngle2;
    Servo dumper;
    Servo doorR;
    Servo doorL;
    DcMotor pullUp1;
    DcMotor pullUp2;
    DcMotor ext;
    Servo climberDumper;
    Servo sideArmL;
    Servo sideArmR;
    float driveMod;

    //Sensing
    TouchSensor extStop;



    @Override
    public void init() {

        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");

        doorR = hardwareMap.servo.get("doorR");
        doorL = hardwareMap.servo.get("doorL");
        collect = hardwareMap.dcMotor.get("collect");
        armAngle1 = hardwareMap.servo.get("armAngle1");
        armAngle2 = hardwareMap.servo.get("armAngle2");
        dumper = hardwareMap.servo.get("dumper");
        pullUp1 = hardwareMap.dcMotor.get("pullUp1");
        pullUp2 = hardwareMap.dcMotor.get("pullUp2");
        ext = hardwareMap.dcMotor.get("ext");
        climberDumper = hardwareMap.servo.get("climberDumper");
        sideArmL = hardwareMap.servo.get("sideArmL");
        sideArmR = hardwareMap.servo.get("sideArmR");
        extStop = hardwareMap.touchSensor.get("extStop");

        ext.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        ext.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        armextended = true;
        oldarm = 0;
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
        if(gamepad1.right_trigger==1){
            driveMod=1.5f;
        }
        else{
            driveMod=1;
        }
        float YVal = gamepad1.left_stick_y;
        float RotVal = gamepad1.right_stick_x;

        // clip the right/left values so that the values never exceed +/- 1
        YPower = Range.clip(YVal, -1, 1);
        rotPower = Range.clip(RotVal, -1, 1);

        float FRpower = -YPower + rotPower;
        float BRpower = -YPower + rotPower;
        float FLpower = YPower + rotPower;
        float BLpower = YPower + rotPower;

        FRpower = Range.clip(FRpower, -1, 1)/driveMod;
        FLpower = Range.clip(FLpower, -1, 1)/driveMod;
        BRpower = Range.clip(BRpower, -1, 1)/driveMod;
        BLpower = Range.clip(BLpower, -1, 1)/driveMod;

        // write the values to the motors
        fr.setPower(FRpower);
        br.setPower(BRpower);
        fl.setPower(FLpower);
        bl.setPower(BLpower);

    }

    public void attachments() {

        //collection in
        if (gamepad2.right_bumper) {
            collect.setPower(1);
            //collection out
        }
        else if (gamepad2.left_bumper) {
            collect.setPower(-1);
            //resting
        }
        else collect.setPower(0);


        //dumping

        if (gamepad2.dpad_right) {
            dumper.setPosition(0);
            doorR.setPosition(0.4);
        }
        else {
            dumper.setPosition(0.25);
            doorR.setPosition(1);
        }


        //mountain climber release
        if(gamepad1.b){
            sideArmR.setPosition(1);
        }
        else{
            sideArmR.setPosition(0);
        }

        if(gamepad1.x){
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

        //tension the pullUp motor
        if (gamepad2.a) {
            pullUp1.setPower(-0.2);
            pullUp2.setPower(0.2);
        }

        //brings arm in
        if (gamepad2.left_trigger!=0){
            pullUp1.setPower(-0.68);
            pullUp2.setPower(0.68);
            armextended = false;
            ext.setPower(-1);
        }
        //sends arm out
        if(ext.getCurrentPosition()<= oldarm){
            armextended = true;
        }

        else if (gamepad2.right_trigger!=0) {
            ext.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            //ext.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            ext.setPower(1);
            if(armextended) {
                pullUp1.setPower(0.11);
                pullUp2.setPower(-0.11);
            }
        }
        else {
            ext.setPower(0);
            pullUp1.setPower(0);
            pullUp2.setPower(0);
        }

        if (extStop.isPressed() && gamepad2.left_trigger!=0) {
            ext.setPower(0);
            pullUp1.setPower(0);
            pullUp2.setPower(0);
        }


        //arm angle

        armAngle2.setPosition(gamepad2.left_stick_y / 2 + 0.5);
        armAngle1.setPosition(gamepad2.left_stick_y / 2 + 0.5);


    }
    double clip(double variable, double min, double max) {
        if (variable < min)
            variable = min;

        if (variable > max)
            variable = max;

        return variable;
    }
}

