package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

/**
 * Created by August on 3/16/2016.
 */
public class AutonomousTest extends AdvancedMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        while (true){
            beaconL.setPosition(0.7);
            beaconR.setPosition(0.2);
            sleep(1);
        }
    }
}
