package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

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

    //Drive
    final int RED = -1;
    final int BLUE = 1;
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
    float yPower;
    /**
     * the rotational power
     */
    float xPower;
    /**
     * the arm's last value
     */

    //Scoring
    /**
     * the motor that powers the collector
     */
    DcMotor collect;
    /**
     * the servo which controls the sliding dumpingBlock block
     */
    Servo dumpingBlock;
    /**
     * the right hopper door control servo
     */
    Servo doorR;
    /**
     * the left hopper door control servo
     */
    Servo doorL;
    /**
     * makes the hook on the end of the arm go out for hanging
     */
    Servo armHook;
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
    DcMotor armAngleMotor;
    /**
     * the servo that controls the climber dumping arm for the blue side
     */
    Servo climberDumper;
    /**
     * left side arm control servo
     */
    Servo sideArmL;
    /**
     * Arm that presses the beacon
     */
    Servo beacon;
    /**
     * right side arm control servo
     */
    Servo sideArmR;
    /**
     * lock1 mech for the hanging
     */
    Servo lock1;
    /**
     * second lock1 mech
     */
    Servo lock2;
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
    float frPower;
    /**
     * power of {@link #br} which can be altered
     */
    float brPower;
    /**
     * power of {@link #fl} which can be altered
     */
    float flPower;
    /**
     * power of {@link #bl} which can be altered
     */
    float blPower;
    /**
     * the value of {@link #gamepad1} joystick 1 y
     * later written to {@link #yPower}
     */
    float yVal;
    /**
     * the value of {@link #gamepad1} joystick 2 x
     * latter written to {@link #xPower}
     */
    float xVal;
    /**
     * this is pressed when the arm is fully retracted
     * it is used by {@link #armControl()}
     */
    TouchSensor extStop;

    boolean wasDown=false;
    boolean lockDown=false;

    /**
     * this maps all of our variables to the hardware
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
        dumpingBlock = hardwareMap.servo.get("dumper");
        pullUp1 = hardwareMap.dcMotor.get("pullUp1");
        pullUp2 = hardwareMap.dcMotor.get("pullUp2");
        armAngleMotor = hardwareMap.dcMotor.get("ext");
        climberDumper = hardwareMap.servo.get("climberDumper");
        sideArmL = hardwareMap.servo.get("sideArmL");
        sideArmR = hardwareMap.servo.get("sideArmR");
        beacon = hardwareMap.servo.get("beacon");
        extStop = hardwareMap.touchSensor.get("extStop");
        armHook = hardwareMap.servo.get("armHook");
        lock1 = hardwareMap.servo.get("lock1");
        lock2 = hardwareMap.servo.get("lock2");

        armAngleMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        armAngleMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        pullUp2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        pullUp1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);


        pullUp1.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        pullUp2.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        lock1.setPosition(1);
        lock2.setPosition(0);
        climberDumper.setPosition(0);
        beacon.setPosition(0);
        sideArmL.setPosition(0.8);
        sideArmR.setPosition(0.05);
    }

    /**
     * all this does is starts the runs the functions that actually process inputs and do the driving
     * @see #drive()
     * @see #attachments()
     */
    @Override
    public void loop() {
        pullUp1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        pullUp2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        attachments();
        drive();
        hang();

        telemetry.addData("Ext", "" + armAngleMotor.getCurrentPosition());
        telemetry.addData("PullUp1", "" + pullUp1.getCurrentPosition());
    }

    /**
     * there are no special stop events
     */
    @Override
    public void stop() {

    }

    /**
     * this processes inputs directly related to moving the wheels
     * @see #readInput()
     * @see #processInput()
     * @see #slowRobot()
     * @see #powerRobot()
     */
    private void drive() {
        readInput();
        processInput();
        slowRobot();
        powerRobot();
    }

    /**
     * this processes all inputs that do not affect the movement of the wheels
     * @see #collector()
     * @see #dumping()
     * @see #climberDumper()
     * @see #armControl()
     * @see #angleArm()
     */
    public void attachments() {
        collector();
        dumping();
        climberDumper();
        sideArm();
        armControl();
        hook();
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
            collect.setPower(-0.4);
            //resting
        } else collect.setPower(0);
    }


    /**
    * pressing {@link #gamepad2} dpad right while blue or {@link #gamepad2} dpad left while red
     * slides {@link #dumpingBlock} across the hopper, bringing the debris with it
    * as this happens the proper release door ({@link #doorR} or {@link #doorL})  opens
    */

    int moveCount = 0;
    int oldCount = 51;


    final int MINCOUNT =  51;
    final int MAXMOVECOUNT = 250;

    private void dumping() {
        telemetry.addData("Moving", "move: " + moveCount + "old: " + oldCount);

        if (side == BLUE) {
            if (gamepad2.dpad_right) {
                dumpingBlock.setPosition(0);
                doorR.setPosition(0.3);
                moveCount++;

                if(moveCount > MAXMOVECOUNT){
                    moveCount = MAXMOVECOUNT;
                }

                oldCount = moveCount;
            }
            else if (gamepad2.dpad_left) {
                dumpingBlock.setPosition(1);
            }
            else { //default position

                if(moveCount >0){
                    dumpingBlock.setPosition(1);
                    moveCount--;
                }
                else {
                    dumpingBlock.setPosition(0.5);
                    oldCount = MINCOUNT;
                }
                if((oldCount - moveCount) > 50) {
                    doorR.setPosition(0.85);
                    doorL.setPosition(0.15);
                }
            }
        }
        else if(side == RED) {
            if (gamepad2.dpad_left) {
                moveCount++;
                dumpingBlock.setPosition(1);
                doorL.setPosition(0.7);

                if(moveCount > MAXMOVECOUNT){
                    moveCount = MAXMOVECOUNT;
                }
                oldCount = moveCount;
            }
            else if (gamepad2.dpad_right) {
                dumpingBlock.setPosition(0);
            }
            else{
                if(moveCount > 0){
                    dumpingBlock.setPosition(0);
                    moveCount--;
                }
                else {
                    dumpingBlock.setPosition(0.5);
                    oldCount = MINCOUNT;
                }
                if((oldCount - moveCount) > 50) {
                    doorL.setPosition(0.15);
                    doorR.setPosition(0.95);
                }
            }
        }
    }

    /**
     * dumps the climber if the {@link #gamepad2} y button is pressed
     */
    private void climberDumper() {
        // climber dumpingBlock
            if (gamepad2.y) {
                climberDumper.setPosition(1);
            }
            else {
                climberDumper.setPosition(0.1);
                beacon.setPosition(1);
            }
    }


    /**
     * runs everything involving extending and retracting the main dumpingBlock arm {@link #armAngleMotor} and synchronizing it with {@link #pullUp1} and {@link #pullUp2}
     * the arm is extended by {@link #gamepad2} right trigger and retracted by {@link #gamepad2} left trigger
     * {@link #gamepad2} a button tightens the arm
     * if the arm is fully retracted, the arm cannot retract more became {@link #extStop} is pressed
     */

    private void armControl() {
        /**
         * if the left trigger is pressed, the arm is retracted
         */
        //brings arm in
        if (gamepad1.y) {
            if (gamepad1.dpad_down && !wasDown) {
                wasDown = true;
                lockDown = !lockDown;
            }
            if (!gamepad1.dpad_down && wasDown) {
                wasDown = false;
            }
            if (lockDown) {
                lock1.setPosition(0.1);
                lock2.setPosition(1);
            } else {
                lock1.setPosition(1);
                lock2.setPosition(0);
            }
            if (lock2.getPosition()>0.9)
                climberDumper.setPosition(0.5);
        }


        //sends arm in
        if (gamepad2.b) {
            pullUp1.setPower(-1);
            pullUp2.setPower(1);
        }
        else if (gamepad2.a) {
            pullUp1.setPower(1);
            pullUp2.setPower(-1);
        }
        else if (gamepad2.left_trigger != 0) {
            sideArmL.setPosition(0.5);
            sideArmR.setPosition(0.5);
            pullUp1.setPower(gamepad2.left_trigger);
            pullUp2.setPower(-gamepad2.left_trigger);
        }
        else if (gamepad1.right_trigger==1 && fr.getPower()>0) {
            if (Math.abs(pullUp1.getCurrentPosition()) < 2500) {
                pullUp1.setPower(-1);
                pullUp2.setPower(1);
            } else {
                pullUp1.setPower(0);
                pullUp2.setPower(0);
            }
        }
        /**
         * if the right trigger is pressed, the arm is extended
         */
        else if (gamepad2.right_trigger != 0) {
            sideArmL.setPosition(0.5);
            sideArmR.setPosition(0.5);
            pullUp1.setPower(-gamepad2.right_trigger);
            pullUp2.setPower(gamepad2.right_trigger);
        }
        else if (!gamepad1.y) {
            pullUp1.setPower(0);
            pullUp2.setPower(0);
        }
    }

    /**
     * sets the angle of the arm
     * this is controlled by {@link #gamepad2} left stick y axis
     */
    private void angleArm() {
        if(gamepad2.left_stick_y > .03) {
            armAngleMotor.setPower(1);
        }
        else if (gamepad2.left_stick_y < -.03) {
            armAngleMotor.setPower(-1);
        }
        else {
            armAngleMotor.setPower(0);
        }
    }

    /**
     * Makes the hook go out
     */
    private void hook(){
        if (gamepad1.left_bumper){
            armHook.setPosition(0.6);
        }
        else {
            armHook.setPosition(0.3);
        }
    }


    private void sideArm(){
        if (fr.getPower()>0) {
            if (side == 1) {
                if (gamepad1.right_bumper || gamepad1.right_trigger != 0) {
                    sideArmL.setPosition(0.8);
                    sideArmR.setPosition(1);//0.8
                }
                else {
                    sideArmL.setPosition(0.8);
                    sideArmR.setPosition(0.05);
                }
            }
            else if (side == -1) {
                if (gamepad1.right_bumper || gamepad1.right_trigger != 0) {
                    sideArmL.setPosition(0);//0.1
                    sideArmR.setPosition(0.05);
                }
                else {
                    sideArmL.setPosition(0.8);
                    sideArmR.setPosition(0.05);
                }
            }
        }
        else {
            sideArmL.setPosition(0.8);
            sideArmR.setPosition(0.05);
        }
    }


    private void hang(){
        if (gamepad1.y) {
            pullUp1.setPower(1);
            pullUp2.setPower(-1);
            //switched armAngleMotor less thn to greater than
            if (armAngleMotor.getCurrentPosition()>0) {
                armAngleMotor.setPower(-.5);
            }
            else {
                armAngleMotor.setPower(0);
            }

            fr.setPower(1);
            br.setPower(1);
            fl.setPower(-1);
            bl.setPower(-1);

            telemetry.addData("hang", "" + armAngleMotor.getCurrentPosition());

        }
    }



    /**
     * receives input from joystick and writes it to variables
     * {@link #gamepad1} left stick y axis controls forwards and backwards
     * {@link #gamepad1} right stick x axis controls rotation
     */
    private void readInput() {
        yVal = gamepad1.left_stick_y;
        xVal = gamepad1.right_stick_x;
    }

    /**
     * processes input to form a complete power for each wheel
     * it also  the input values to avoid errors
     */
    private void processInput() {
        yPower = Range.clip(yVal, -1, 1);
        xPower = Range.clip(xVal, -1, 1);

        /**
         * combines the rotation and speed together
         */
        frPower = yPower + xPower;
        brPower = yPower + xPower;
        flPower = -yPower + xPower;
        blPower = -yPower + xPower;
    }

    /**
     *sets power
     *also clips the speed to avoid errors and slows down the robot if required
     * @see #fr
     * @see #br
     * @see #fl
     * @see #bl
     */
    private void powerRobot() {
        if (!gamepad1.y){
            fr.setPower(Range.clip(frPower, -1, 1) * driveMod);
            br.setPower(Range.clip(brPower, -1, 1) * driveMod);
            fl.setPower(Range.clip(flPower, -1, 1) * driveMod);
            bl.setPower(Range.clip(blPower, -1, 1) * driveMod);
        }
    }

    /**
     * if {@link #gamepad1} right trigger is pressed, it causes the robot to slow down
     */
    private void slowRobot() {

        if(gamepad1.right_trigger == 1) {

            if (fr.getPower() > 0) {
                driveMod = 1f;

            }
            else if (fr.getPower() < 0) {
                driveMod = 0.5f;

            }
        }
        else
            driveMod = 1;
    }
}

