package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by August on 2/7/2016.
 */
public class rampDetection extends LinearOpMode {
    Cameracontroller cameracontroller;

    public rampDetection(){
        super();
        cameracontroller = new Cameracontroller(this);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        cameracontroller.startCam();
        waitForStart();
        while (true){
            telemetry.addData("left ", cameracontroller.leftRed() + " right: " + cameracontroller.rightRed());
        }
    }
}
