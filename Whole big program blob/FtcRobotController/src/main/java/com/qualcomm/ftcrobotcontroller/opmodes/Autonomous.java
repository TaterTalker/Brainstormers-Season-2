package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.Drive;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.Drive;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.rotTracker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.Turn;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.rotTracker;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by August on 10/10/2015.
 */
public class Autonomous extends OpMode {
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor FR;
    private int v_state = 0;
    int turning = 0, turningOld = 0;
    int isTurning = 0;


    public void init() {
        rotTracker.startTracking();
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
    }
    @Override public void start ()

    {

        super.start ();
        reset_drive_encoders ();

    } // start

    void setdrivepower(double left_power, double right_power) {

        if (FL != null&&BL != null)
        {
            FL.setPower(-left_power);
            BL.setPower(left_power);
        }
        if (FR != null&&BR != null)
        {
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
                (Math.abs(BR.getCurrentPosition()) > rightd)
                ;


    } // have_encoders_reached
    boolean have_drive_encoders_reset (){

        return (FL.getCurrentPosition()==0 &&
                FR.getCurrentPosition()==0 &&
                BL.getCurrentPosition()==0 &&
                BR.getCurrentPosition()==0 );
    }


    public void loop() {
//        if (gamepad1.x && turning == turningOld) {
//            //Turn.Turn(90, 1);
//            isTurning=1;
//        }
//        telemetry.addData("Is Turning", "" + String.format("%s", isTurning));
//        telemetry.addData("degs", "" + String.format("%s", rotTracker.degs));
//
//        turningOld=turning;

        switch (v_state) {

            case 0:

                reset_drive_encoders();

                v_state++;

                break;

            case 1:

                run_using_encoders();
                // Start the drive wheel motors at full power
                setdrivepower(1.0f, 1.0f);

                if (have_drive_encoders_reached(40000, 40000)) {
                    reset_drive_encoders();
                    setdrivepower(0.0f, 0.0f);
                    v_state++;
                }
                break;
            case 2:
                if (have_drive_encoders_reset ())
                {
                    v_state++;
                }
                break;
            default:
                break;


        }
        telemetry.addData ("Text", "State: " + v_state);
    }

}