package com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by August on 10/25/2015.
 */
public class Turn extends rotTracker{
    static DcMotor FL;
    static DcMotor BR;
    static DcMotor BL;
    static DcMotor FR;

    public static void Turn(float rot, float speed){
        degs=0;

        while(Math.abs(rot)>Math.abs(degs)){
            FL.setPower(speed*(rot/Math.abs(rot)));
            BL.setPower(speed*(rot/Math.abs(rot)));
            FR.setPower(-speed * (rot / Math.abs(rot)));
            BR.setPower(-speed * (rot / Math.abs(rot)));
        }
    }

    Turn Turn = new Turn();
}