package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by August on 12/18/2015.
 */
public class ultraTest extends OpMode {
    UltrasonicSensor ultra1;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor FR;
    double lowestval=255;
    double highestval=0;
    double currVal;
    const ULTRASONIC = "ultra1";

    @Override
    public void init() {
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        ultra1 = hardwareMap.ultrasonicSensor.get(ULTRASONIC);
        if (ultra1 != null) {
            telemetry.addData("Sensor works:" + ULTRASONIC);
        }
        else {
            telemetry.addData("sensor sux" + ULTRASONIC);
        }
    }

    @Override
    public void loop() {
        currVal=readFixedUltra(ultra1);
        if(currVal!=0 && currVal<lowestval)
            lowestval=currVal;
        else if(currVal!=255 && currVal>highestval)
            highestval=currVal;
        telemetry.addData("Highest Value", ""+highestval);
        telemetry.addData("Lowest Value", ""+lowestval);
    }

    double readFixedUltra(UltrasonicSensor sensor){
        double val = 0;
        for(int i=0;i<10;i++) {
            val+=sensor.getUltrasonicLevel();
        }
        val/=10;
        return val;
    }
}
