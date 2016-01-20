package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * contains all the code to run TeleOp, it has no inherent side
 */
public abstract class TeleOpOctopus extends OpMode {


    //drive
    /**
     * front left drive motor
     */
    DcMotor fl;
    /**
     * back right drive motor
     */
    DcMotor br;
    /**
     * back left drive motor
     */
    DcMotor bl;
    /**
     * front right drive motor
     */
    DcMotor fr;
    /**
     * the forwards backwards power
     */
    float YPower;
    /**
     * the rotational power
     */
    float rotPower;
    /**
     * set as true when the arm is fully extended
     */
    boolean armextended = false;
    /**
     * the arm's last value
     */
    int oldarm;

    //scoring
    /**
     * the motor that powers the collector
     */
    DcMotor collect;
    /**
     * arm angler 1
     */
    Servo armAngle1;
    /**
     * arm angler 2
     */
    Servo armAngle2;
    /**
     * the servo which controls the sliding dumper block
     */
    Servo dumper;
    /**
     * the right hopper door control servo
     */
    Servo doorR;
    /**
     * the left hopper door control servo
     */
    Servo doorL;
    /**
     * pull up control motor 1
     */
    DcMotor pullUp1;
    /**
     * pull up control motor 2
     */
    DcMotor pullUp2;
    /**
     * extender control motor
     */
    DcMotor ext;
    /**
     * the servo that controls the climber dumping arm
     */
    Servo climberDumper;
    /**
     * left side arm control servo
     */
    Servo sideArmL;
    /**
     * right side arm control servo
     */
    Servo sideArmR;
    /**
     * how much the robot should be slowed by
     * higher=slower
     */
    float driveMod;
    /**
     * informs the program whether it is red or blue
     * 1=blue
     * -1=red
     */
    int side;
    /**
     * power of {@link #fr} which can be altered
     */
    float FRpower;
    /**
     * power of {@link #br} which can be altered
     */
    float BRpower;
    /**
     * power of {@link #fl} which can be altered
     */
    float FLpower;
    /**
     * power of {@link #bl} which can be altered
     */
    float BLpower;
    /**
     * the value of {@link #gamepad1} joystick 1 y
     * later written to {@link #YPower}
     */
    float YVal;
    /**
     * the value of {@link #gamepad1} joystick 2 x
     * latter written to {@link #rotPower}
     */
    float rotVal;
    //Sensing
    /**
     * this is pressed when the arm is fully retracted
     * it is used by {@link #processArm()}
     */
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
     * @see #drive()
     * @see #attachments()
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
     * @see #getInput()
     * @see #processInput()
     * @see #slowRobot()
     * @see #setPower()
     */
    private void drive() {
        getInput();
        processInput();
        slowRobot();
        setPower();
    }

    /**
     * this processes all inputs that do not affect the movement of the wheels
     * @see #collector()
     * @see #dumping()
     * @see #climberDumper()
     * @see #processArm()
     * @see #angleArm()
     */
    public void attachments() {
        collector();
        dumping();
        climberDumper();
        processArm();
        angleArm();
    }

    /**
     * Runs the collector
     * in is {@link #gamepad2} right bumper
     * out is {@link #gamepad2}left bumper
     */
    private void collector() {
        if (gamepad2.right_bumper) {
            collect.setPower(1);
            //collection out
        } else if (gamepad2.left_bumper) {
            collect.setPower(-1);
            //resting
        } else collect.setPower(0);
    }

    /**
    * pressing {@link #gamepad2} dpad right while blue or {@link #gamepad2} dpad left while red
     * slides {@link #dumper} across the hopper, bringing the debris with it
    * as this happens the proper release door ({@link #doorR} or {@link #doorL})  opens
    */
    private void dumping() {
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
    }

    /**
     * dumps the climber if the {@link #gamepad2} y button is pressed
     */
    private void climberDumper() {
        // climber dumper
        if (gamepad2.y) {
            climberDumper.setPosition(1);
        } else {
            climberDumper.setPosition(0);
        }
    }

    /**
     * runs everything involving extending and retracting the main dumper arm
     * the arm is extended by {@link #gamepad2} right trigger and retracted by {@link #gamepad2} left trigger
     * {@link #gamepad2} a button tightens the arm
     * if the arm is fully retracted, the arm cannot retract more became {@link #extStop} is pressed
     */
    private void processArm() {
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
         * if the arm is fully retracted this stops it from trying to retract further
         * it is controlled by touch sensor {@link #extStop}
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
         * tightens the pull up if the arm is fully extended and gamepad 2 dpad right is pressed
         */
        if (gamepad2.dpad_right && armextended) {
            pullUp1.setPower(-0.05);
            pullUp2.setPower(0.05);
        }
    }

    /**
     * sets the angle of the arm
     * this is controlled by {@link #gamepad2} left stick y axis
     */
    private void angleArm() {
        armAngle2.setPosition(gamepad2.left_stick_y / 2 + 0.5);
        armAngle1.setPosition(gamepad2.left_stick_y / 2 + 0.5);
    }

    /**
     * receives input from joystick and writes it to variables
     * {@link #gamepad1} left stick y axis controls forwards and backwards
     * {@link #gamepad1} right stick x axis controls rotation
     */
    private void getInput() {
        YVal = gamepad1.left_stick_y;
        rotVal = gamepad1.right_stick_x;
    }

    /**
     * processes input to form a complete power for each wheel
     * it also  the input values to avoid errors
     */
    private void processInput() {
        YPower = Range.clip(YVal, -1, 1);
        rotPower = Range.clip(rotVal, -1, 1);

        /**
         * combines the rotation and speed together
         */
        FRpower = -YPower + rotPower;
        BRpower = -YPower + rotPower;
        FLpower = YPower + rotPower;
        BLpower = YPower + rotPower;
    }

    /**
     *sets power
     *also clips the speed to avoid errors and slows down the robot if required
     * @see #fr
     * @see #br
     * @see #fl
     * @see #bl
     */
    private void setPower() {
        fr.setPower(Range.clip(FRpower, -1, 1)/driveMod);
        br.setPower(Range.clip(BRpower, -1, 1)/driveMod);
        fl.setPower(Range.clip(FLpower, -1, 1)/driveMod);
        bl.setPower(Range.clip(BLpower, -1, 1)/driveMod);
    }

    /**
     * if {@link #gamepad1} right trigger is pressed, it causes the robot to slow down
     */
    private void slowRobot() {
        if (gamepad1.right_trigger == 1) {
            driveMod = 1.2f;
        } else {
            driveMod = 1;
        }
    }
}

