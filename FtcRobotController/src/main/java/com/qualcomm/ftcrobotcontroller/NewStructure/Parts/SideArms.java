package com.qualcomm.ftcrobotcontroller.NewStructure.Parts;

import com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes.AdafruitIMUmethods;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ethan on 4/5/2016.
 */
public class SideArms {
    Servo sideArmL;
    Servo sideArmR;
    OpMode opMode;
    int side;


    public SideArms (int side,OpMode varopMode ){

        opMode = varopMode;
        this.side = side;
        sideArmL = opMode.hardwareMap.servo.get("sideArmL");
        sideArmR = opMode.hardwareMap.servo.get("sideArmR");
    }

   public void initSideArms() {

       sideArmL.setPosition(0.8);
       sideArmR.setPosition(0.05);
   }

    public void sideArm( AdafruitIMUmethods gyro,double gyroOffset, DcMotor fr){
        if (fr.getPower()>0) {
            if (side == 1) {
                if (opMode.gamepad1.right_trigger != 0  || (gyro.getRoll() + gyroOffset) > 3.5 || opMode.gamepad1.y) {
                    sideArmL.setPosition(0.8);
                    sideArmR.setPosition(1);
                }
                else {
                    sideArmL.setPosition(0.8);
                    sideArmR.setPosition(0.05);
                }
            }
            else if (side == -1) {
                if (opMode.gamepad1.right_trigger != 0 || (gyro.getRoll() + gyroOffset) > 3.5 || opMode.gamepad1.y) {
                    sideArmL.setPosition(0);
                    sideArmR.setPosition(0.05);
                }
                else {
                    sideArmL.setPosition(0.8);
                    sideArmR.setPosition(0.05);
                }
            }
        }
        else if (opMode.gamepad2.right_trigger !=0 || opMode.gamepad2.left_trigger !=0){
            sideArmL.setPosition(0.5);
            sideArmR.setPosition(0.5);
        }
        else {
            sideArmL.setPosition(0.8);
            sideArmR.setPosition(0.05);
        }
    }

    public void setSideArmLpos(float pos){
        sideArmL.setPosition(pos);
    }
    public void setSideArmRpos(float pos){
        sideArmR.setPosition(pos);
    }


}
