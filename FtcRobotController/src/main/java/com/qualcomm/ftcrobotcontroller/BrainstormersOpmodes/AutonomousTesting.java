package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

/**
 * Created by August on 2/13/2016.
 */
public class AutonomousTesting extends AvancedMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        getRobotConfig();
        beacon.setPosition(0);
        advancedgyro = new AdafruitIMUmanager();

        advancedgyro.init();
        sleep(5000);
        telemetry.addData("init", " done");
        waitForStart();


            beacon.setPosition(0.4); //right
            sleep(2000);
            beacon.setPosition(0.1); //left
            sleep(2000);


    }
}
