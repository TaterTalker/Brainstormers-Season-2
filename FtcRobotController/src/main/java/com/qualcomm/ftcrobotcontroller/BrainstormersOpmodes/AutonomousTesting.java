package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

/**
 * Created by August on 2/13/2016.
 */
public class AutonomousTesting extends AdvancedMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        getRobotConfig();
        while (!isStarted()) {
            adaFruitGyro.initIMU();
        }
        sleep(100);
        pivot(60,-1,1);
        sleep(500);
        pivot(60,1,1);
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
