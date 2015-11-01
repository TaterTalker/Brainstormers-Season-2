package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

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
        reset_drive_encoders();
    }

    //Loop through the state machine completing each task.
    public void loop() {

        switch (v_state) {

            //Reset Motors.
            case 0:

                reset_drive_encoders();

                v_state++;

                break;

            //Travel 40000 distance.
            case 1:

                run_using_encoders();
                // Start the drive wheel motors at full power
                set_drive_power(1.0f, 1.0f);

                if (have_drive_encoders_reached(40000, 40000)) {
                    reset_drive_encoders();
                    set_drive_power(0.0f, 0.0f);
                    v_state++;
                }
                break;

            //Reset Motors.
            case 2:

                if (have_drive_encoders_reset()) {
                    v_state++;
                }
                break;
            //End the state machine.
            default:

                break;
        }
        telemetry.addData("Text", "State: " + v_state);
    }

    //The drive function.
    void set_drive_power(double left_power, double right_power) {

        if (FL != null && BL != null) {
            FL.setPower(-left_power);
            BL.setPower(left_power);
        }
        if (FR != null && BR != null) {
            BR.setPower(right_power);
            FR.setPower(-right_power);
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

    boolean have_drive_encoders_reached(double leftd, double rightd) {

        telemetry.addData("Encoder", "Encoderdistance: " + FL.getCurrentPosition());
        return (Math.abs(FL.getCurrentPosition()) > leftd) &&
                (Math.abs(BL.getCurrentPosition()) > leftd) &&
                (Math.abs(FR.getCurrentPosition()) > rightd) &&
                (Math.abs(BR.getCurrentPosition()) > rightd);
    }

    boolean have_drive_encoders_reset() {

        return (FL.getCurrentPosition() == 0 &&
                FR.getCurrentPosition() == 0 &&
                BL.getCurrentPosition() == 0 &&
                BR.getCurrentPosition() == 0);
    }
}