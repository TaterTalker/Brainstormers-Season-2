package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by August on 4/2/2016.
 */
public class CameraDebrisCounter extends AutonomousBuildingBlocks implements Runnable {
    int row=5;
    @Override
    public void runOpMode() throws InterruptedException {
        frontCam.startFrontCam();
        getRobotConfig();
        runUsingEncoders();
        collector.setPower(0);
        waitForStart();
        run();
        sleep(100000);
        while(true){
            if (gamepad1.dpad_up){
                row-=1;
            }
            if (gamepad1.dpad_down){
                row+=1;
            }
            telemetry.addData("row", row);
            int[] colors = getRowColors(row);
            telemetry.addData("rbg values", " " + colors[0] + " " + colors[1] + " " + colors[2]);
            telemetry.addData("width ", frontCam.width);
            telemetry.addData("height ", frontCam.height);
            waitOneFullHardwareCycle();
        }
    }

    int[] getRowColors(int row){
        //try {
            frontCam.convertImage();
        //} catch (Exception e){}

        int[] tmpArray = {0,0,0};
        int pixelX=1;
        while (pixelX<frontCam.width) {
            tmpArray[0]+=frontCam.getPixelColors(pixelX,row)[0];
            tmpArray[1]+=frontCam.getPixelColors(pixelX,row)[1];
            tmpArray[2]+=frontCam.getPixelColors(pixelX,row)[2];
            pixelX++;
        }
        return tmpArray;
    }
    @Override
    public void run() {
        int countInTarget=0;
        boolean isComplete=false;
        while(!isComplete){
            runUsingEncoders();
            collector.setPower(1);
            int[] colors = getRowColors(120);
            if (colors[0]>18000){
                countInTarget++;
            }
            else if (countInTarget>0&&colors[2]<15000){
                countInTarget--;
            }

            if (countInTarget>20){
                isComplete=true;
            }

        }
        collector.setPower(-1);
    }
}
