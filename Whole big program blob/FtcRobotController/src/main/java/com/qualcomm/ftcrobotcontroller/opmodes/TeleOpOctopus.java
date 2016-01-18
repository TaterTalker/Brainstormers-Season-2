package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
public abstract class TeleOpOctopus extends OpMode {


    //drive
    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    DcMotor fr;
    float YPower, rotPower, YPower2;
    boolean armextended = false;
    int oldarm;

    //scoring
    DcMotor collect;
    Servo armAngle1;
    Servo armAngle2;
    Servo dumper;
    Servo doorR;
    Servo doorL;
    DcMotor pullUp1;
    DcMotor pullUp2;
    DcMotor ext;
    Servo climberDumper;
    Servo sideArmL;
    Servo sideArmR;
    float driveMod;
    int side;

    //Sensing
    TouchSensor extStop;

    /**
     * this maps all of our variables to the hardware
     *
     * @param sideInput this is given by our separate programs that reference this file so that autonomous knows weather it is red or blue. 1=blue, -1=red
     */
    public void init(int sideInput) {
        side=sideInput;

        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");

        doorR = hardwareMap.servo.get("doorR");
        doorL = hardwareMap.servo.get("doorL");
        collect = hardwareMap.dcMotor.get("collect");
        armAngle1 = hardwareMap.servo.get("armAngle1");
        armAngle2 = hardwareMap.servo.get("armAngle2");
        dumper = hardwareMap.servo.get("dumper");
        pullUp1 = hardwareMap.dcMotor.get("pullUp1");
        pullUp2 = hardwareMap.dcMotor.get("pullUp2");
        ext = hardwareMap.dcMotor.get("ext");
        climberDumper = hardwareMap.servo.get("climberDumper");
        sideArmL = hardwareMap.servo.get("sideArmL");
        sideArmR = hardwareMap.servo.get("sideArmR");
        extStop = hardwareMap.touchSensor.get("extStop");

        ext.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        ext.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        armextended = false;
        oldarm = 0;
    }

    /**
     * all this does is starts the runs the functions that actually process inputs and do the driving
     */
    @Override
    public void loop() {
        drive();
        attachments();
        telemetry.addData("Encoder Val", "" + ext.getCurrentPosition());
        telemetry.addData("armextended", "" + armextended);
    }

    /**
     * there are no special stop events
     */
    @Override
    public void stop() {

    }

    /**
     * this processes inputs directly related to moving the wheels
     */
    private void drive() {
        /**
         * if right trigger is pressed, it causes the robot to slow down
         */
        if(gamepad1.right_trigger==1){
            driveMod=1.2f;
        }
        else{
            driveMod=1;
        }

        /**
         * gets values from the joysticks
         */
        float YVal = gamepad1.left_stick_y;
        float RotVal = gamepad1.right_stick_x;

        /**
         * clip the right/left values so that the values never exceed +/- 1
         */
        YPower = Range.clip(YVal, -1, 1);
        rotPower = Range.clip(RotVal, -1, 1);

        /**
         * combines the rotation and speed together
         */
        float FRpower = -YPower + rotPower;
        float BRpower = -YPower + rotPower;
        float FLpower = YPower + rotPower;
        float BLpower = YPower + rotPower;

        /**
         * this makes sure that the power stays within the exeptable bounds
         * it also applies a speed modifier if required
         */
        FRpower = Range.clip(FRpower, -1, 1)/driveMod;
        FLpower = Range.clip(FLpower, -1, 1)/driveMod;
        BRpower = Range.clip(BRpower, -1, 1)/driveMod;
        BLpower = Range.clip(BLpower, -1, 1)/driveMod;

        // write the values to the motors
        /**
         * writes the final values to the motors
         */
        fr.setPower(FRpower);
        br.setPower(BRpower);
        fl.setPower(FLpower);
        bl.setPower(BLpower);

    }

    /**
     * this processes all inputs that do not affect the movement of the wheels
     */
    public void attachments() {

        //collection in
        /**
         * Runs the collector
         * in is right bumper
         * out is left bumper
         */
        if (gamepad2.right_bumper) {
            collect.setPower(1);
            //collection out
        } else if (gamepad2.left_bumper) {
            collect.setPower(-1);
            //resting
        } else collect.setPower(0);


        //dumping

        /**
         * pressing a button causes the dumper to slide to the other side
         * as this happens the release door opens
         * if the robot is blue the trigger is the D-pad right, otherwise it is D-pad left
         */

        if (side == 1) {

            if (gamepad2.dpad_right) {
                dumper.setPosition(0);
                doorR.setPosition(0.2);
            } else { //default position
                dumper.setPosition(0.25);
                doorR.setPosition(1);
                doorL.setPosition(0);
            }
        } else {
            if (gamepad2.dpad_left) {
                dumper.setPosition(0.25);
                doorL.setPosition(0.8);
            } else { //default position
                dumper.setPosition(0);
                doorL.setPosition(0);
                doorR.setPosition(1);
            }
        }


            //mountain climber release
        /*if(gamepad1.left_bumper){
            sideArmR.setPosition(0);
        }
        else{
            sideArmR.setPosition(0.5);
        }*/

        /**
         * dumps the climber if the y button is pressed
         */
            // climber dumper
            if (gamepad2.y) {
                climberDumper.setPosition(1);
            } else {
                climberDumper.setPosition(0);
            }

            //arm

        /**
         * if the left trigger is pressed, the arm is retracted
         */
            //brings arm in
            if (gamepad2.left_trigger != 0) {
                pullUp1.setPower(-0.8);
                pullUp2.setPower(0.8);
                armextended = false;
                ext.setPower(-1);
                oldarm = ext.getCurrentPosition();
            }
            //sends arm out

            /**
             * if the right trigger is pressed, the arm is extended
             */
            else if (gamepad2.right_trigger != 0) {
                ext.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                //ext.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                ext.setPower(1);
                if (ext.getCurrentPosition() <= oldarm && ext.getCurrentPosition() > 6500) {
                    armextended = true;
                }
                oldarm = ext.getCurrentPosition();
                if (!armextended) {
                    pullUp1.setPower(0.16);
                    pullUp2.setPower(-0.16);
                } else {
                    pullUp1.setPower(0);
                    pullUp2.setPower(0);
                }
            } else {
                ext.setPower(0);
                pullUp1.setPower(0);
                pullUp2.setPower(0);
                oldarm = ext.getCurrentPosition();
            }

        /**
         * if the arm is fully retracted
         * this stops it from retracting further
         */
            if (extStop.isPressed() && gamepad2.left_trigger != 0) {
                ext.setPower(0);
                pullUp1.setPower(0);
                pullUp2.setPower(0);
                oldarm = ext.getCurrentPosition();
                ext.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            }

        /**
         * if a is pressed, this tightens the pull up motor
         */
            //tension the pullUp motor
            if (gamepad2.a) {
                pullUp1.setPower(-0.2);
                pullUp2.setPower(0.2);
            }
        /**
         * tightens the pull up if the arm is fully extended
         */
            if (gamepad2.dpad_right && armextended) {
                pullUp1.setPower(-0.05);
                pullUp2.setPower(0.05);
            }

        /**
         * sets the angle of the arm
         */
            armAngle2.setPosition(gamepad2.left_stick_y / 2 + 0.5);
            armAngle1.setPosition(gamepad2.left_stick_y / 2 + 0.5);

        }

    /**
     * takes a variable and returns the variable
     * if the variable is outside the one of the bounds, instead it returns that bound
     * @param variable the variable to be clipped
     * @param min the minimum exeptable value
     * @param max the maximum exeptable value
     * @return returns the variable, exept it is inside the exeptable vlues
     */
    double clip(double variable, double min, double max) {
        if (variable < min)
            variable = min;

        if (variable > max)
            variable = max;

        return variable;
    }
}

