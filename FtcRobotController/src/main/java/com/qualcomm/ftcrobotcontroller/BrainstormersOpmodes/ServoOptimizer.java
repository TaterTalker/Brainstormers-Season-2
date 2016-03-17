package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by August on 2/13/2016.
 */
public class ServoOptimizer extends AdvancedMethods {
    Servo lock1;
    Servo lock2;
    @Override
    public void runOpMode() throws InterruptedException {
        getRobotConfig();
        while (!adaFruitGyro.initDone) {
            adaFruitGyro.initIMU();
        }

        boolean oldUp=false;
        boolean oldDown=false;
        double position=0.5;
        lock1 = hardwareMap.servo.get("lock1");
        lock2 = hardwareMap.servo.get("lock2");
        waitForStart();
        while (true) {
            if (gamepad1.dpad_up!=oldUp&&oldUp==false){
                position+=0.1;
            }
            if (gamepad1.dpad_down!=oldDown&& oldDown ==false){
                position-=0.1;
            }
            lock1.setPosition(position);
            lock2.setPosition(position);
            oldUp=gamepad1.dpad_up;
            oldDown=gamepad1.dpad_down;
            telemetry.addData("position ", position);
            sleep(1);
        }
    }
    public void turnTest() throws InterruptedException {
        long start= System.currentTimeMillis();
        newGyroTurn(36, 0.5);
        newGyroTurn(-11, 0.5); //-11
        newGyroTurn(63, 0.5); //63
        newGyroTurn(-102, 0.5); //-102
        newGyroTurn(-55, 0.5); //-55
        newGyroTurn(173, 0.5); // 173
        long end=System.currentTimeMillis();
        telemetry.addData("timing", Long.toString(end-start));
        sleep(150000);
    }

}
