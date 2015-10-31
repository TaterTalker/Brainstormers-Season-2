package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.Drive;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.Drive;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.rotTracker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.Turn;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.rotTracker;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by August on 10/10/2015.
 */
public class Autonomous extends OpMode{
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor FR;
    int turning=0, turningOld=0;
    int isTurning=0;

    public void init() {
        rotTracker.startTracking();
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
    }

    public void loop() {
        if (gamepad1.x && turning == turningOld) {
            //Turn.Turn(90, 1);
            isTurning=1;
        }
        telemetry.addData("Is Turning", "" + String.format("%s", isTurning));
        telemetry.addData("degs", "" + String.format("%s", rotTracker.degs));

        turningOld=turning;
    }
}
