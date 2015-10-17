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

    void drive(int distance, float speed) {
        initMotors();
        FL.setTargetPosition(distance);
        BR.setTargetPosition(distance);
        BL.setTargetPosition(distance);
        FR.setTargetPosition(distance);

    }

    private void initMotors() {
        if (FL != null)
        {
            FL.setChannelMode
                    ( DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }
        if (FR != null)
        {
            FR.setChannelMode
                    ( DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }
        if (BL != null)
        {
            BL.setChannelMode
                    ( DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }
        if (BR != null)
        {
            BR.setChannelMode
                    ( DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }
    }
}
