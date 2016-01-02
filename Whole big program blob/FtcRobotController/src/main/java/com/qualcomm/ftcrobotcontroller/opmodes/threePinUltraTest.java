package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;

/**
 * Created by August on 1/2/2016.
 */
public class threePinUltraTest extends LinearOpMode{
    AnalogInput ultra;
    AnalogOutput ultraSig;
    @Override
    public void runOpMode() throws InterruptedException {
        ultra=hardwareMap.analogInput.get("ultra1");
        ultraSig=hardwareMap.analogOutput.get("ultra1");
        waitForStart();
        ultraSig.setAnalogOutputFrequency(0);
        while(true){
            ultraSig.setAnalogOutputFrequency(1);
            sleep(1);
            ultraSig.setAnalogOutputFrequency(0);
            telemetry.addData("value", " " + ultra.getValue());
            waitOneFullHardwareCycle();
        }

    }

    void read3pinUltra(){
    }
}
