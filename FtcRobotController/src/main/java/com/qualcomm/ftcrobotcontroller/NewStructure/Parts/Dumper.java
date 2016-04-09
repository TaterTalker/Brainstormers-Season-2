package com.qualcomm.ftcrobotcontroller.NewStructure.Parts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ethan on 4/5/2016.
 */
public class Dumper {
    Servo doorR;
    Servo doorL;
    Servo dumpingBlock;
    int side;
    OpMode opMode;

    final int RED = -1;
    final int BLUE = 1;

    public Dumper(int side , OpMode varopmode){
        this.side = side;
        opMode = varopmode;
        dumpingBlock = opMode.hardwareMap.servo.get("dumper");
        doorR = opMode.hardwareMap.servo.get("doorR");
        doorL = opMode.hardwareMap.servo.get("doorL");

    }

    public void startDumper(){

        doorR.setPosition(0.85);
        doorL.setPosition(0.15);
        dumpingBlock.setPosition(0.5);
    }

    int moveCount = 0;
    int oldCount = 51;
    int moveCountAllClear = 0;
    int oldCountAllClear = 51;

    final int MINCOUNT =  51;
    final int MAXMOVECOUNT = 250;

    public void resetDumpingBlock(){
        if (side==-1){
            dumpingBlock.setPosition(0.35);
        } else {
            dumpingBlock.setPosition(0.55);
        }
    }
    /**
     * configures our dumping system based on what team we're on
     */
    public void dumping() {
        opMode.telemetry.addData("Moving", "move: " + moveCount + "old: " + oldCount);
        if (side == BLUE) {
            if (opMode.gamepad2.dpad_right || opMode.gamepad2.right_stick_x > .15) {
                doorR.setPosition(0.3);
            }
            else {
                doorR.setPosition(0.85);
                doorL.setPosition(0.15);
            }
        }
        else if (side == RED) {
            if (opMode.gamepad2.dpad_left || opMode.gamepad2.right_stick_x < -.15) {
                doorL.setPosition(0.7);
            }
            else {
                doorR.setPosition(0.85);
                doorL.setPosition(0.15);
            }
        }


        if (side == BLUE) {
            if (opMode.gamepad2.dpad_right) {
                dumpingBlock.setPosition(0);
                moveCount++;

                if(moveCount > MAXMOVECOUNT){
                    moveCount = MAXMOVECOUNT;
                }

                oldCount = moveCount;
            } else if (opMode.gamepad2.dpad_left) {
                dumpingBlock.setPosition(1);

            }
            else {
                if(moveCount > 0 && opMode.gamepad2.right_stick_x < .15){
                    dumpingBlock.setPosition(1);
                    moveCount--;
                }
                else {
                    dumpingBlock.setPosition(0.5);
                    oldCount = MINCOUNT;
                }
            }


        } else if(side == RED) {
            if (opMode.gamepad2.dpad_left) {
                moveCount++;
                dumpingBlock.setPosition(1);

                if(moveCount > MAXMOVECOUNT){
                    moveCount = MAXMOVECOUNT;
                }
                oldCount = moveCount;
            }
            else if (opMode.gamepad2.dpad_right) {
                dumpingBlock.setPosition(0);
            }
            else {
                if(moveCount > 0 && opMode.gamepad2.right_stick_x > -.15){
                    dumpingBlock.setPosition(0);
                    moveCount--;
                }
                else {
                    dumpingBlock.setPosition(0.5);
                    oldCount = MINCOUNT;
                }
            }
        }
    }





}
