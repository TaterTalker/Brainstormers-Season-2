package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by August on 12/5/2015.
 */
public class turnTest extends OpMode {
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor FR;
    int loopCount=0;
    boolean turnComplete=false;
    GyroSensor gyroSensor;
    int i=0;

    @Override
    public void init() {
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        gyroSensor = hardwareMap.gyroSensor.get("G1");
        gyroSensor.calibrate();
    }

    @Override
    public void loop() {
        switch (i){
            case 0:
                resetGyro();
                break;
            case 1:
                turnWithGyro(90);
                break;
            default:
                break;
        }
    }


    void resetGyro(){
        telemetry.addData("heading: ", "" + gyroSensor.getHeading());
        if(gyroSensor.getHeading()!=0){
            gyroSensor.calibrate();
        }
        else {
            pause(1000);
        }
    }

    void turnWithGyro(int degrees){
        telemetry.addData("heading: ", "" + gyroSensor.getHeading());
        if (degrees<0){
            degrees+=360;
        }

        int curDegs = gyroSensor.getHeading();
        if(turnComplete==false) {

            if (degrees > 180) {
                if (degrees<curDegs) {
                    FR.setPower(-1);
                    BR.setPower(-1);
                    FL.setPower(-1);
                    BL.setPower(-1);
                }
                else turnComplete=true;
            } else if(degrees>curDegs){

                FR.setPower(1);
                BR.setPower(1);
                FL.setPower(1);
                BL.setPower(1);
            }
            else turnComplete=true;
        }

       if(turnComplete==true){
           turnComplete=false;
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            i++;
        }
    }

    void pause(float pauseAmount) {
        if(loopCount>pauseAmount) {
            loopCount = 0;
            i++;
        }
        loopCount++;
    }
}
