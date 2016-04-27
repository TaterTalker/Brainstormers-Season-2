package com.qualcomm.ftcrobotcontroller.NewStructure.Parts;

import com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes.AdafruitIMUmethods;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ethan on 4/5/2016.
 */
public class Arm {
    OpMode opMode;
    DcMotor pullUp1;
    DcMotor pullUp2;
    DcMotor armAngleMotor;
    Servo lock1;
    Servo lock2;

    boolean wasDown=false;
    boolean lockDown=false;


    public Arm (OpMode varopmode ){
        opMode =varopmode;
        pullUp1 = opMode.hardwareMap.dcMotor.get("pullUp1");
        pullUp2 = opMode.hardwareMap.dcMotor.get("pullUp2");
        armAngleMotor = opMode.hardwareMap.dcMotor.get("ext");
        lock1 = opMode.hardwareMap.servo.get("lock1");
        lock2 = opMode.hardwareMap.servo.get("lock2");



    }

    public void runArmAngleEncoders(){
        armAngleMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }
    public void resetArmAngleEncoders(){
        armAngleMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public void runArmEncoders(){
        pullUp2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        pullUp1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }
    public void resetArmEncoders(){
        pullUp1.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        pullUp2.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public void setArmPower(float power){
        pullUp1.setPower(power);
        pullUp2.setPower(-power);

    }
    public void setArmAnglePower(float power){
        armAngleMotor.setPower(power);
    }
    public void setLockDown(){
        lock1.setPosition(0);
        lock2.setPosition(1);
    }
    public void setLockUp(){
        lock1.setPosition(1);
        lock2.setPosition(0);
    }

    public float getArmAnglePos(){
        return armAngleMotor.getCurrentPosition();
    }
    public float getArmPos(){
        return pullUp1.getCurrentPosition();
    }
    public double getLockPos(){
        return lock2.getPosition();
    }



    public void startTeleArm(){

        resetArmAngleEncoders();
        runArmAngleEncoders();
        resetArmEncoders();
        runArmAngleEncoders();

        lock1.setPosition(1);
        lock2.setPosition(0);
    }


    public void angleArm() {
        if (!opMode.gamepad1.y) {
            if ((opMode.gamepad2.left_stick_y > .03 && getArmAnglePos()<0) || opMode.gamepad2.a) {
                setArmAnglePower(1);
            } else if (opMode.gamepad2.left_stick_y < -.03) {
                setArmAnglePower(-1);
            } else {
                setArmAnglePower(0);
            }
        }
    }


    public void armControl( AdafruitIMUmethods gyro,double gyroOffset, DcMotor fr) {
        if (opMode.gamepad1.y) {
            if (opMode.gamepad1.dpad_down && !wasDown) {
                wasDown = true;
                lockDown = !lockDown;
            }
            if (!opMode.gamepad1.dpad_down && wasDown) {
                wasDown = false;
            }
            if (lockDown) {
                setLockDown();
            } else {
                setLockUp();
            }

        }
        else if (opMode.gamepad2.left_trigger != 0) {
            setArmPower(opMode.gamepad2.left_trigger);
        }
        else if ((opMode.gamepad1.right_trigger==1  || (gyro.getRoll() + gyroOffset) > 2 )&& fr.getPower()>0) {
            if (Math.abs(getArmPos()) < 2500  ) {

                setArmPower(-1);
            } else if (opMode.gamepad2.right_trigger == 0 && opMode.gamepad2.left_trigger == 0){
                setArmPower(0);
            }
        }
        else if (opMode.gamepad2.right_trigger != 0) {
            setArmPower(-opMode.gamepad2.right_trigger);
        } else if (!opMode.gamepad1.y) {
            setArmPower(0);
        }
    }
    public void hang() {
        if (opMode.gamepad1.y) {
            setArmPower(1);
            if (getArmAnglePos() < 0) {
                setArmAnglePower(1);
            } else {
                setArmAnglePower(0);
            }
        }
    }
}
