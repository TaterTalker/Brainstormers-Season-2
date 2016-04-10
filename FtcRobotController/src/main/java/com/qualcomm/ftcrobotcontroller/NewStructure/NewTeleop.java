package com.qualcomm.ftcrobotcontroller.NewStructure;

//import com.qualcomm.robotcore.eventloop.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by ethan on 4/8/2016.
 */
public abstract class NewTeleop extends OpMode {

    TeleOpBot teleOpBot;
    float oldTime;
    double gyroOffset=0;

    public void init(int side) {
        teleOpBot = new TeleOpBot(side, this);
    }

    @Override
    public void loop() {
        if (System.currentTimeMillis()-oldTime<100){
            teleOpBot.climberDumper.setPosition(1);
        } else{
            teleOpBot.climberDumper.setPosition(0.5);
        }
        teleOpBot.arm.runArmEncoders();

        attachments();
        drive();
        hang();

        if (teleOpBot.arm.getLockPos()>0.9)
            teleOpBot.beaconL.setPosition(0.5);
        else {
            teleOpBot.beaconL.setPosition(0);
        }
        telemetry.addData("Arm Angle", "" + teleOpBot.arm.getArmAnglePos());
        telemetry.addData("PullUp1", "" + teleOpBot.arm.getArmPos());
        telemetry.addData("min", "min" + teleOpBot.minpullup);
    }

    public void start(){
        oldTime = System.currentTimeMillis();;
    }

    private void drive() {
        teleOpBot.wheelBase.setInput(gamepad1.right_stick_x,gamepad1.left_stick_y );
        slowRobot();
    }

    public void attachments() {
        collector();
        teleOpBot.dumper.dumping();
        teleOpBot.sideArms.sideArm(teleOpBot.adaFruitGyro,gyroOffset,teleOpBot.wheelBase.getFr());
        teleOpBot.arm.armControl(teleOpBot.adaFruitGyro, gyroOffset,teleOpBot.wheelBase.getFr());
        hook();
        teleOpBot. arm.angleArm();
        allClearSymbol();

    }

    public void collector() {
        if (gamepad2.right_bumper) {
            teleOpBot.collector.setPower(1);
            gyroOffset = -teleOpBot.adaFruitGyro.getRoll();
            //collection out
        } else if (gamepad2.left_bumper) {
            teleOpBot.collector.setPower(-0.4);
            gyroOffset = -teleOpBot.adaFruitGyro.getRoll();
            //resting
        } else teleOpBot.collector.setPower(0);
    }
    /**
     * Makes the hook on the end of the arm go out to allow us to hang
     */
    private void hook(){
        if (gamepad1.left_bumper){
            teleOpBot.armHook.setPosition(0.6);
        }
        else {
            teleOpBot.armHook.setPosition(0.2);
        }
    }
    /**
     * we autonomized hang to put less stress on our drivers and hopefully let us hang 100% of the time
     */
    private void hang() {
        if (gamepad1.y) {
            teleOpBot.arm.hang();
            teleOpBot.wheelBase.driveBackwards();

            telemetry.addData("Arm Angle", "" + teleOpBot.arm.getArmAnglePos());
        }
    }

    private void slowRobot() {
        telemetry.addData("GyroPitch", " " + (teleOpBot.adaFruitGyro.getRoll() + gyroOffset));
        if(gamepad1.right_trigger == 1 || ((teleOpBot.adaFruitGyro.getRoll() + gyroOffset) > 6 && (teleOpBot.adaFruitGyro.getRoll() + gyroOffset) < 50)) {

            if (teleOpBot.wheelBase.getFr().getPower() > 0) {
                teleOpBot.wheelBase.setDriveMod(1f);

            }
            else if (teleOpBot.wheelBase.getFr().getPower() < 0) {
                teleOpBot.wheelBase.setDriveMod(0.3f);

            }
        }
        else
            teleOpBot.wheelBase.setDriveMod(1);
    }

    private void allClearSymbol() {
        if (gamepad2.x) {
            teleOpBot.allClear.setPosition(1);
        } else if (gamepad2.y){
            teleOpBot.allClear.setPosition(0);

        } else {
            teleOpBot.allClear.setPosition(0.5);
        }

    }

}

