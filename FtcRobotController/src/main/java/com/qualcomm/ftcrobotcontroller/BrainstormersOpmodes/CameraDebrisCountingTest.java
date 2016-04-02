package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by August on 4/2/2016.
 */
public class CameraDebrisCountingTest extends AutonomousBuildingBlocks {
    boolean upButtonOld=false;
    boolean downButtonOld=false;
    int row=1;

    @Override
    public void runOpMode() throws InterruptedException {
        frontCam.startFrontCam();
        waitForStart();
        while(true){
            if (gamepad1.dpad_up&&!upButtonOld){
                row+=1;
            }
            if (gamepad1.dpad_down&&!downButtonOld){
                row-=1;
            }
            upButtonOld=gamepad1.dpad_up;
            downButtonOld=gamepad1.dpad_down;
            telemetry.addData("row", row);
            int[] colors = getRowColors(row);
            telemetry.addData("rbg values", " " + colors[0] + " " + colors[1] + " " + colors[2]);
        }
    }

    int[] getRowColors(int row){
        int[] tmpArray = {0,0,0};
        int pixelX=1;
        while (pixelX<frontCam.width) {
            telemetry.addData("x val", pixelX);
            tmpArray[0]+=frontCam.getPixelColors(pixelX,row)[0];
            tmpArray[0]+=frontCam.getPixelColors(pixelX,row)[1];
            tmpArray[0]+=frontCam.getPixelColors(pixelX,row)[2];
            pixelX++;
        }
        return tmpArray;
    }
}
