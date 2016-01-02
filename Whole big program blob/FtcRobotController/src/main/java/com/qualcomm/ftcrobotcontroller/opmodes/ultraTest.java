package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by August on 12/18/2015.
 */
public class ultraTest extends OpMode {
    AnalogInput ultra1;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor FR;
    double lowestval=255;
    double highestval=0;
    double currVal;
    public static final String ULTRASONIC = "ultra1";

    @Override
    public void init() {
        ultra1 = hardwareMap.analogInput.get(ULTRASONIC);
        if (ultra1 != null) {
            telemetry.addData("sensor works", "Sensor1: " + ULTRASONIC);
        }
        else {
            telemetry.addData("sensor sux", "Sensor1: " + ULTRASONIC);
        }
    }

    @Override
    public void loop() {
        currVal=hardwareMap.analogInput.get(ULTRASONIC);
        if(currVal!=0 && currVal<lowestval)
            lowestval=currVal;
        else if(currVal!=255 && currVal>highestval) {
            highestval = currVal;
        }
        telemetry.addData("Highest Value", ""+highestval);
        telemetry.addData("Lowest Value", ""+lowestval);
        telemetry.addData("Ultra Value: ", currVal)
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
