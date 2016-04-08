package com.qualcomm.ftcrobotcontroller.NewStructure;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by ethan on 4/5/2016.
 */
public class TeleOpBot extends Robot {

    public TeleOpBot(int side , OpMode varopMode){
        super(side, varopMode);
        arm.startTeleArm();
        dumper.startDumper();
        sideArms.initSideArms();

        climberDumper.setPosition(0.5);
        beaconR.setPosition(1);
        beaconL.setPosition(0) ;
        armHook.setPosition(0.3);
        climberDumper.setPosition(0);
        allClear.setPosition(0.5);

        while (!adaFruitGyro.initDone) {
            adaFruitGyro.initIMU();
        }
        opMode.telemetry.addData("Initialization Done", "");
    }

    long oldTime;
    public void functloop(){

        if (System.currentTimeMillis()-oldTime<100){
            climberDumper.setPosition(1);
        } else{
            climberDumper.setPosition(0.5);
        }
        arm.runArmEncoders();

        attachments();
        drive();
        hang();

        if (arm.getLockPos()>0.9)
            beaconL.setPosition(0.5);
        else {
            beaconL.setPosition(0);
        }
       opMode.telemetry.addData("Arm Angle", "" + arm.getArmAnglePos());
       opMode.telemetry.addData("PullUp1", "" + arm.getArmPos());
       opMode.telemetry.addData("min", "min" + minpullup);
    }

    public void startBot(){
        oldTime = System.currentTimeMillis();
    }

    private void drive() {
        wheelBase.setInput(opMode.gamepad1.right_stick_x,opMode.gamepad1.left_stick_y );
        slowRobot();
    }

    public void attachments() {
        collector();
        dumper.dumping();
        sideArms.sideArm(adaFruitGyro,gyroOffset,wheelBase.getFr());
        arm.armControl(adaFruitGyro, gyroOffset,wheelBase.getFr());
        hook();
        arm.angleArm();
        allClearSymbol();

    }

    public void collector() {
        if (opMode.gamepad2.right_bumper) {
            collector.setPower(1);
            gyroOffset = -adaFruitGyro.getRoll();
            //collection out
        } else if (opMode.gamepad2.left_bumper) {
            collector.setPower(-0.4);
            gyroOffset = -adaFruitGyro.getRoll();
            //resting
        } else collector.setPower(0);
    }
    /**
     * Makes the hook on the end of the arm go out to allow us to hang
     */
    private void hook(){
        if (opMode.gamepad1.left_bumper){
            armHook.setPosition(0.6);
        }
        else {
            armHook.setPosition(0.2);
        }
    }
    /**
     * we autonomized hang to put less stress on our drivers and hopefully let us hang 100% of the time
     */
    private void hang(){
            arm.hang();
            wheelBase.driveBackwards();

            opMode.telemetry.addData("Arm Angle", "" + arm.getArmAnglePos());
    }

    private void slowRobot() {
        opMode.telemetry.addData("GyroPitch", " " + (adaFruitGyro.getRoll() + gyroOffset));
        if(opMode.gamepad1.right_trigger == 1 || ((adaFruitGyro.getRoll() + gyroOffset) > 6 && (adaFruitGyro.getRoll() + gyroOffset) < 50)) {

            if (wheelBase.getFr().getPower() > 0) {
                wheelBase.setDriveMod(1f);

            }
            else if (wheelBase.getFr().getPower() < 0) {
                wheelBase.setDriveMod(0.3f);

            }
        }
        else
            wheelBase.setDriveMod(1);
    }

    private void allClearSymbol() {
        if (opMode.gamepad2.x) {
            allClear.setPosition(1);
        } else if (opMode.gamepad2.y){
            allClear.setPosition(0);

        } else {
            allClear.setPosition(0.5);
        }

    }
}
