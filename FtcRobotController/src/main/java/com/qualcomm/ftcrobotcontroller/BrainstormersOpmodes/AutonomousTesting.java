package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

/**
 * Created by August on 2/13/2016.
 */
public class AutonomousTesting extends AdvancedMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        getRobotConfig();
        while (!adaFruitGyro.initDone) {
            adaFruitGyro.initIMU();
        }
        waitForStart();
        while (true) {
            if (gamepad1.x) {
                climberDumper.setPosition(1);
            }
            else {
                climberDumper.setPosition(0);
            }

            if (gamepad1.a) {
                beaconR.setPosition(1);
            }
            else {
                beaconR.setPosition(0);
            }

            if (gamepad1.y) {
                beaconL.setPosition(1);
            }
            else {
                beaconL.setPosition(0);
            }
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
