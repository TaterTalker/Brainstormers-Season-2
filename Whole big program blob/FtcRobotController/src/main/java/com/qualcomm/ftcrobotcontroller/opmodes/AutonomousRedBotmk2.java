package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by August on 10/10/2015.
 */
public class AutonomousRedBotmk2 extends AutonomousLinearBotmk2 {
    /**
     * runs {@link #runOpMode(int)} set up for red
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(-1);
    }
}