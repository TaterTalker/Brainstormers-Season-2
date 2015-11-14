package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.Drive;

/**
 * Created by August on 10/10/2015.
 */
public class Autonomous extends OpMode {
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor FR;

    private int v_state = 0;

    //Map the motors.
    public void init() {

        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
    }

    //Start the program and reset the encoders.
    @Override
    public void start() {

        super.start();
    }

    //Loop through the state machine completing each task.
    public void loop() {

        switch (v_state) {

            //Reset Motors.
            case -1:
                //null state
                //to be used if action should only be performed once
                break;

            case 0:
                drive(4000, 4000, 1);
                break;

            //Travel 40000 distance.
            case 1:
                break;

            //Reset Motors.
            default:

                break;
        }
        telemetry.addData("Text", "State: " + v_state);
    }

    //The drive function.


    void drive(float rightDistance, float leftDistance, float speed) {
        run_using_encoders();
        // Start the drive wheel motors at full power

        if (hasLeftReached(leftDistance) && hasRightReached(rightDistance)) {
            setLeftPower(0);
            setRightPower(0);
            reset_drive_encoders();
            v_state++;
        }

        else {
            setLeftPower(speed);
            setRightPower(speed);
        }
    }

    void reset_drive_encoders() {

        FL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        FR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        BL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        BR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    void run_using_encoders() {

        FL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        FR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    boolean hasLeftReached(double leftd) {

        return (Math.abs(FL.getCurrentPosition()) > leftd) &&
                (Math.abs(BL.getCurrentPosition()) > leftd);
    }

    boolean hasRightReached(double rightd) {

        return (Math.abs(FR.getCurrentPosition()) > rightd) &&
                (Math.abs(BR.getCurrentPosition()) > rightd);
    }

    boolean have_drive_encoders_reset() {

        return (FL.getCurrentPosition() == 0 &&
                FR.getCurrentPosition() == 0 &&
                BL.getCurrentPosition() == 0 &&
                BR.getCurrentPosition() == 0);
    }

    void setLeftPower(double power) {
        power=clip(power,-1,1);
        FL.setPower(-power);
        BL.setPower(power);
    }

    void setRightPower(double power) {
        power=clip(power,-1,1);
        FR.setPower(-power);
        BR.setPower(power);
    }

    double clip(double variable, double min, double max){
        if(variable<min)
            variable=min;

        if (variable>max)
            variable=max;

        return variable;
    }
}