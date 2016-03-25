package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by August on 2/7/2016.
 */
public class rampDetection extends LinearOpMode {
    BackCameraController cameraController;

    public rampDetection(){
        super();
        cameraController = new BackCameraController(this);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        cameraController.startBackCam();
        waitForStart();
        while (true){
            telemetry.addData("left ", cameraController.getLeftRed() + " right: " + cameraController.getRightRed());
        }
    }
}
