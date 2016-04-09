package com.qualcomm.ftcrobotcontroller.NewStructure;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by ethan on 4/5/2016.
 */
public class TeleOpBot extends Robot {

    public TeleOpBot(int side , OpMode varopMode){
        super(side, varopMode);
        arm.startTeleArm();
        dumper.startDumper();
        sideArms.initSideArms();

        climberDumper.setPosition(0.5);
        beaconR.setPosition(1);
        beaconL.setPosition(0) ;
        armHook.setPosition(0.3);
        climberDumper.setPosition(0);
        allClear.setPosition(0.5);

        while (!adaFruitGyro.initDone) {
            adaFruitGyro.initIMU();
        }
        opMode.telemetry.addData("Initialization Done", "");
    }



}
