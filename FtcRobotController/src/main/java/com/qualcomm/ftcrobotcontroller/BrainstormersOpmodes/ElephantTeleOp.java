package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by dan on 3/12/16.
 */
public class ElephantTeleOp extends OpMode{
        DcMotor fl;
        DcMotor br;
        DcMotor bl;
        DcMotor fr;
        float yVal;
        float rotVal;
        float yPower;
        float rotPower;
        float FRpower;
        float FLpower;
        float BRpower;
        float BLpower;


    @Override
    public void init() {
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
        telemetry.addData("elephants", "are low key lame");
    }

    public void loop() {
            drive();
        }


        private void drive() {
            recieveInput();
            processInput();
            applyPower();
        }


        private void recieveInput() {
            yVal = gamepad1.left_stick_y;
            rotVal = gamepad1.right_stick_x;
        }

        /**
         * processes input to form a complete power for each wheel
         * it also  the input values to avoid errors
         */
        private void processInput() {
            yPower = Range.clip(yVal, -1, 1);
            rotPower = Range.clip(rotVal, -1, 1);

            /**
             * combines the rotation and speed together
             */
            FRpower = yPower + rotPower;
            BRpower = yPower + rotPower;
            FLpower = -yPower + rotPower;
            BLpower = -yPower + rotPower;
        }

        /**
         *sets power
         *also clips the speed to avoid errors and slows down the robot if required
         * @see #fr
         * @see #br
         * @see #fl
         * @see #bl
         */
        private void applyPower() {
            fr.setPower(Range.clip(FRpower, -1, 1));
            br.setPower(Range.clip(BRpower, -1, 1));
            fl.setPower(Range.clip(FLpower, -1, 1));
            bl.setPower(Range.clip(BLpower, -1, 1));
        }
}






