package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

/**
 * Created by August on 2/13/2016.
 */
public class AutonomousTesting extends AdvancedMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        getRobotConfig();
        while (!isStarted()) {
            initIMU();
        }
        PIdrive(4000,0.5);
        sleep(150000);
    }
    public void turnTest() throws InterruptedException {
        long start= System.currentTimeMillis();
        newGyroTurn(36, 0.5);
        newGyroTurn(25, 0.5); //-11
        newGyroTurn(88, 0.5); //63
        newGyroTurn(-170, 0.5); //-102
        newGyroTurn(135, 0.5); //-55
        newGyroTurn(-52, 0.5); // 173
        long end=System.currentTimeMillis();
        telemetry.addData("timing", Long.toString(end-start));
        sleep(150000);
    }
}
