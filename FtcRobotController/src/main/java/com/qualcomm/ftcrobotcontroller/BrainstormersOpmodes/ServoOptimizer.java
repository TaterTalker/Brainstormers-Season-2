package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by August on 2/13/2016.
 */
public class ServoOptimizer extends AdvancedMethods {
    Servo beaconR;
    Servo beaconL;
    @Override
    public void runOpMode() throws InterruptedException {
        getRobotConfig();
        while (!adaFruitGyro.initDone) {
            adaFruitGyro.initIMU();
        }

        boolean oldUp=false;
        boolean oldDown=false;
        double position=0.5;
        beaconR = hardwareMap.servo.get("beacon right");
        beaconL = hardwareMap.servo.get("beacon left");
        waitForStart();
       climberDumper.setPosition(0.575);

        lock1.setPosition(0);
        lock2.setPosition(1);

        sleep (10000);
//        while (true) {
//            beaconL.setPosition(.9);
//            beaconR.setPosition(.1);
//        }
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
