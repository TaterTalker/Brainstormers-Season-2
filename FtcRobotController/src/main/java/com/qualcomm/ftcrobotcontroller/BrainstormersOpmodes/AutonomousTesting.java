package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

/**
 * Created by August on 2/13/2016.
 */
public class AutonomousTesting extends AvancedMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        getRobotConfig();
        while (!isStarted()) {
            initIMU();
        }
        long start= System.currentTimeMillis();
        newturnTo(36, 0.5);
        newturnTo(25, 0.5);
        newturnTo(88, 0.5);
        newturnTo(-170, 0.5);
        newturnTo(135, 0.5);
        newturnTo(-52, 0.5);
        long end=System.currentTimeMillis();
        telemetry.addData("timing", Long.toString(end - start));
        sleep(150000);

    }
}
