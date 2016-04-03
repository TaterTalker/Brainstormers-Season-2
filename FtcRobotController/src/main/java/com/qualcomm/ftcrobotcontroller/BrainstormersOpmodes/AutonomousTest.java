package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

/**
 * Created by August on 3/16/2016.
 */
public class AutonomousTest extends AdvancedMethods {
    CameraDebrisCounter debrisCounter = new CameraDebrisCounter();
    @Override
    public void runOpMode() throws InterruptedException {
        frontCam.startFrontCam();
        getRobotConfig();
        runUsingEncoders();
        collector.setPower(0);
        waitForStart();
    }
}
