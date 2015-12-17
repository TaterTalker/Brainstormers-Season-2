package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by August on 12/5/2015.
 */
public class turnTest extends LinearOpMode {
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor FR;
    int loopCount = 0;
    boolean turnComplete = false;
    GyroSensor gyroSensor;
    int i = 0;
    int lastgyro;


    @Override
    public void runOpMode() throws InterruptedException {
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        gyroSensor = hardwareMap.gyroSensor.get("G1");
        gyroSensor.calibrate();
        sleep(5000);
        waitForStart();
        turnWithGyro(90);
        sleep(1000);
        turnWithGyro(180);
        sleep(1000);
        turnWithGyro(-270);
    }


    void resetGyro() {
        while(heading()!=0) {
            lastgyro = gyroSensor.getHeading();
        }
    }

    void turnWithGyro(int degrees) throws InterruptedException {
        resetGyro();
        telemetry.addData("heading ", "" + heading());
        if (degrees < 0)
            degrees += 370;


        else
            degrees-=10;

        if (degrees > 180) {
            while (degrees < heading() || heading()<10) {
                telemetry.addData("heading ", "" + heading());
                FR.setPower(-0.5);
                BR.setPower(-0.5);
                FL.setPower(-0.5);
                BL.setPower(-0.5);
                sleep(1);
            }

            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);

        } else {
            while (degrees > heading() || heading()>350) {
                telemetry.addData("heading ", "" + heading());
                FR.setPower(0.5);
                BR.setPower(0.5);
                FL.setPower(0.5);
                BL.setPower(0.5);
                sleep(1);
            }
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        telemetry.addData("Done","");

    }

    void pause(float pauseAmount) {
        if(loopCount>pauseAmount) {
            loopCount = 0;
            i++;
        }
        loopCount++;
    }

    int heading(){
        int head;
        head=gyroSensor.getHeading()-lastgyro;
        if (head<0)
            head+=360;
        return (head);
    }
}
