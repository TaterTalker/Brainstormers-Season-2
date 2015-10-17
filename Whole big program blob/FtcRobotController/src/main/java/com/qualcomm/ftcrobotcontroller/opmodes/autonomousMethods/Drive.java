package com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.ftcrobotcontroller.opmodes.mainDriving;
import android.hardware.Sensor;
import android.hardware.SensorEvent;

/**
 * Created by August on 10/17/2015.
 */
public class Drive {
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor FR;

    boolean drive_using_encoders(double p_left_power, double p_right_power, double p_left_count, double p_right_count) {
        //
        // Assume the encoders have not reached the limit.
        //
        boolean l_return = false;

        //
        // Tell the system that motor encoders will be used.
        //
        if (BR != null)
        {
            BR.setChannelMode( DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
        if (BL != null)
        {
            BL.setChannelMode( DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

        //
        // Start the drive wheel motors at full power.
        //
        if (BL != null)
        {
            BL.setPower (p_left_power);
        }
        if (BR != null)
        {
            BR.setPower (p_left_power);
        }
        if (FL != null)
        {
            FR.setPower (p_right_power);
        }
        if (FR != null)
        {
            FR.setPower (p_right_power);
        }

        //
        // Have the motor shafts turned the required amount?
        //
        // If they haven't, then the op-mode remains in this state (i.e this
        // block will be executed the next time this method is called).
        //
        if (have_drive_encoders_reached(p_left_count, p_right_count)) {
            //
            // Reset the encoders to ensure they are at a known good value.
            //
            reset_drive_encoders();

            //
            // Stop the motors.
            //
            set_drive_power(0.0f, 0.0f);

            //
            // Transition to the next state when this method is called
            // again.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;
    }
}
