package com.qualcomm.ftcrobotcontroller.NewStructure;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by ethan on 4/8/2016.
 */
public class NewTeleop extends OpMode {

    TeleOpBot teleOpBot;

    @Override
    public void init() {
        teleOpBot = new TeleOpBot(-1, this);
    }

    @Override
    public void loop() {

        teleOpBot.functloop();

    }

    public void start(){
        teleOpBot.startBot();
    }
}
