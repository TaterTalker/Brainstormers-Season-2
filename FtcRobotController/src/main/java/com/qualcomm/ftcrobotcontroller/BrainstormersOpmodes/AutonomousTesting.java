package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

/**
 * Created by August on 2/13/2016.
 */
public class AutonomousTesting extends AutonomousMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        getRobotConfig();
        beacon.setPosition(0.5); //right
        sleep(2000);
        beacon.setPosition(0.1); //left


    }
}
