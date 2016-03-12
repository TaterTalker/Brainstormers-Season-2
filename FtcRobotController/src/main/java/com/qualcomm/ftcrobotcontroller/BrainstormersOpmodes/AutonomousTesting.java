package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

/**
 * Created by August on 2/13/2016.
 */
public class AutonomousTesting extends AvancedMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        FR = hardwareMap.dcMotor.get("fr");
        FL = hardwareMap.dcMotor.get("fl");
        BR = hardwareMap.dcMotor.get("br");
        BL = hardwareMap.dcMotor.get("bl");
        advancedgyro = new AdafruitIMUmanager();
        advancedgyro.init();
        sleep(5000);
        waitForStart();
        advancedgyro.start();

        newturnTo(90,0.5);
        sleep(1000);
        PIdrive(1000,0.9);


    }
}
