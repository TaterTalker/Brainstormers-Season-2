package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by August on 10/10/2015.
 */
public abstract class Autonomous extends OpMode {
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    GyroSensor gyroSensor;
    DcMotor FR;
    DcMotor collector;
    Servo climberDumper;
    Servo sideArmL;
    Servo lock;
    Servo sideArmR;
    boolean turnComplete = false;
    Servo debDumper;
    Servo door;
    UltrasonicSensor ultra1;
    private final double TURNRATIO = 18.3;
    private int v_state = 0;
    private double turnChange = 0;
    private int turnDirection = 0;
    private int loopCount = 0;
    private double ultrastate = 0;
    private boolean encoders_have_reset=false;

    //Map the motors.
    public void init() {
        sideArmL = hardwareMap.servo.get("sideArmL");
        sideArmR = hardwareMap.servo.get("sideArmR");
        lock = hardwareMap.servo.get("lock");
        gyroSensor = hardwareMap.gyroSensor.get("G1");
        climberDumper = hardwareMap.servo.get("climberdumper");
        debDumper = hardwareMap.servo.get("debDumper");
        door = hardwareMap.servo.get("door");
        collector= hardwareMap.dcMotor.get ("colmot");
        v_state= 0;
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        ultra1 = hardwareMap.ultrasonicSensor.get("ultra1");
    }

    //Start the program and reset the encoders.
    @Override
    public void start() {
        super.start();
        v_state = 0;
        loopCount = 0;
    }

    //Loop through the state machine completing each task.
    public void loop(int turnDirection, double turnChange) {
        this.turnDirection=turnDirection;
        this.turnChange=turnChange;
        switch (v_state) {

            //Reset Motors.
            case -1:
                //null state
                break;

            case 0:
                reset_drive_encoders();
                sideArmL.setPosition(1);
                sideArmR.setPosition(0);
                climberDumper.setPosition(1);
                collector.setPower(1);
                v_state++;
                break;

            case 1:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                   encoders_have_reset= true;
                    drive(2000, 1);
                }
                break;
            case 2:
                pause (100);
                break;
            case 3:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                     turn(32, -0.5);
                }
                break;
            case 4:
                pause (1);
                break;
            case 5:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    drive(5700, 1);
                }
                break;
            case 6:
                pause (100);
                break;
            case 7:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    turn(50, -0.5);
                }
                break;
            case 8:
                collector.setPower(0);
                pause(1);
                break;
            case 9:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    ultrastate=ultra1.getUltrasonicLevel();
                    driveForever(0.2);
                    if(ultrastate < 10 && ultrastate > 1)
                    {
                        setLeftPower(0);
                        setRightPower(0);
                        v_state++;
                    }
                    else {driveForever(.2);}
                }
                break;
            case 10:
                if(encoders_have_reset|| have_drive_encoders_reset()) {
                    climberDumper.setPosition(0);
                    if (climberDumper.getPosition() < 0.2) {
                        v_state++;
                    }
                }
                break;
            case 11:
                pause(200);
                break;
            case 12:
                if(encoders_have_reset|| have_drive_encoders_reset()) {
                    climberDumper.setPosition(.92);
                    if (climberDumper.getPosition() > 0.72) {
                        reset_drive_encoders();
                        v_state++;
                    }
                }
                break;
            case 13:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    collector.setPower(1);
                    drive(500, -0.5);
                }
                break;
            case 14:
                pause(100);
                break;
            case 15:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                   turn(83, -0.5);
                }
                break;
            case 16:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    drive(1600, 1);
                }
                break;
            case 17:
                pause (100);
                break;
            case 18:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    turn(30, -0.5);
                }
                break;
            case 19:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    drive(1700, 1);
                }
                break;
            case 20:
                pause(100);
                break;
            case 21:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    if (turnDirection==-1)
                        sideArmR.setPosition(1);
                    else
                        sideArmL.setPosition(0);
                    turn(97, -0.5);
                }
                break;
            case 22:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset = true;
                    driveForever(-1);
                    v_state++;
                }
                break;
            case 23:
                pause (300);
                break;
            case 24:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    setLeftPower(0);
                    setRightPower(0);
                    sideArmL.setPosition(1);
                    sideArmR.setPosition(0);
                }
                break;
            default:
        }
        telemetry.addData("Text", "State: " + v_state);
    }

    void pause(float pauseAmount) {
        if(loopCount>pauseAmount) {
            loopCount = 0;
            v_state++;
        }
        else
            loopCount++;
    }
    //The drive with collector function.


    void drive_with_collector(float distance, double speed) {
    // Start the drive wheel motors at full power

        if (hasLeftReached(distance) || hasRightReached(distance)) {
            setLeftPower(0);
            setRightPower(0);
            collector.setPower(0);
            reset_drive_encoders();
            v_state++;
        }

        else {
            run_using_encoders();
            setLeftPower(speed);
            setRightPower(speed);
            collector.setPower(-1);
        }
    }

    void driveRampDown(float distance, double speed) {
        if (hasLeftReached(distance) || hasRightReached(distance)) {
            setLeftPower(0);
            setRightPower(0);
            reset_drive_encoders();
            v_state++;
        }
        else if (isLeftNear(distance) || isRightNear(distance)) {
            run_using_encoders();
            setLeftPower(speed*0.5);
            setRightPower(speed*0.5);
        }
        else {
            run_using_encoders();
            setLeftPower(speed);
            setRightPower(speed);
        }
    }

    void drive(float distance, double speed) {
        // Start the drive wheel motors at full power

        if (hasLeftReached(distance) || hasRightReached(distance)) {
            setLeftPower(0);
            setRightPower(0);
            reset_drive_encoders();
            v_state++;
        }

        else {
            run_using_encoders();
            setLeftPower(speed);
            setRightPower(speed);
        }
    }
    void driveForever(double speed) {
        // Start the drive wheel motors at full power
        run_using_encoders();
        setLeftPower(speed);
        setRightPower(speed);
    }


    void reset_drive_encoders() {

        FL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        FR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        BL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        BR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        encoders_have_reset=false;
    }

    void run_using_encoders() {

        FL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        FR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    boolean hasLeftReached(double leftd) {

        return (Math.abs(FL.getCurrentPosition()) > leftd) &&
                (Math.abs(BL.getCurrentPosition()) > leftd);
    }

    boolean hasRightReached(double rightd) {

        return (Math.abs(FR.getCurrentPosition()) > rightd) &&
                (Math.abs(BR.getCurrentPosition()) > rightd);
    }

    boolean isLeftNear(double leftd) {

        return (Math.abs(FL.getCurrentPosition()) > leftd-500) &&
                (Math.abs(BL.getCurrentPosition()) > leftd-500);
    }

    boolean isRightNear(double rightd) {

        return (Math.abs(FR.getCurrentPosition()) > rightd-500) &&
                (Math.abs(BR.getCurrentPosition()) > rightd-500);
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
        BL.setPower(-power);
    }

    void setRightPower(double power) {
        power=clip(power,-1,1);
        FR.setPower(power);
        BR.setPower(power);
    }

    double clip(double variable, double min, double max) {
        if(variable<min)
            variable=min;

        if (variable>max)
            variable=max;

        return variable;
    }

    void initEncoders(){
        reset_drive_encoders();
        run_using_encoders();
    }

    void turn(double degrees, double power){
        if (power<0)
            degrees=degrees*1.05;
        if(hasLeftReached(degrees*TURNRATIO*turnChange)||hasRightReached(degrees * TURNRATIO*turnChange)) {
                setLeftPower(0);
                setRightPower(0);
                reset_drive_encoders();
                v_state++;
            }
            else{
                run_using_encoders();
            setLeftPower(power*turnDirection);
            setRightPower(-power*turnDirection);
            }
        }

    void turnWithGyro(int degrees){
        if(loopCount<400) {
            loopCount ++;
            return;
        }
        telemetry.addData("heading: ", "" + gyroSensor.getHeading());
        degrees=degrees-10;
        if (degrees<0){
            degrees+=360;
        }

        int curDegs = gyroSensor.getHeading();
        if(turnComplete==false) {

            if (degrees > 180) {
                if (degrees<curDegs) {
                    run_using_encoders();
                    FR.setPower(-0.5);
                    BR.setPower(-0.5);
                    FL.setPower(-0.5);
                    BL.setPower(-0.5);
                }
                else turnComplete=true;
            } else if(degrees>curDegs){
                run_using_encoders();
                FR.setPower(0.5);
                BR.setPower(0.5);
                FL.setPower(0.5);
                BL.setPower(0.5);
            }
            else turnComplete=true;
        }

        if(turnComplete==true){
            turnComplete=false;
            run_using_encoders();
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            v_state++;
            loopCount=0;
        }
    }

    void resetGyro(){
        telemetry.addData("heading: ", "" + gyroSensor.getHeading());
        gyroSensor.calibrate();
        if(gyroSensor.getHeading()==0) {
            v_state++;
        }
    }
}
