package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.Drive;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by August on 10/10/2015.
 */
public abstract class Autonomous extends OpMode {
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor FR;
    DcMotor collector;
    Servo climberDumper;
    UltrasonicSensor ultra1;
    private final double TURNRATIO = 18.3;
    private int v_state = 0;
    private int loopCount = 0;
    private double ultrastate = 0;
    private boolean encoders_have_reset=false;

    //Map the motors.
    public void init() {
        climberDumper = hardwareMap.servo.get("climberdumper");
        v_state= 0;
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        collector = hardwareMap.dcMotor.get("colmot");
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
    public void loop(double turnDirection, double turnChange) {

        switch (v_state) {

            //Reset Motors.
            case -1:
                //null state
                break;

            case 0:
                reset_drive_encoders();
                //climber.setPower(0.5);
                //run_using_encoders();
                v_state++;
                break;

            case 1:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                   encoders_have_reset= true;
                    drive(2000, -1);
                }
                break;
            case 2:
                pause(100);
                break;
            case 3:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    turn(40 * turnChange, -1 * turnDirection);
                    //telemetry.addData("rightTicks", "" + FL.getCurrentPosition());
                    //telemetry.addData("leftTicks", "" + FR.getCurrentPosition());
                }
                break;

            case 4:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    drive(5900, -1);
                }
                break;
            case 5:
                pause(100);
                break;
            case 6:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    turn(118 * turnChange, 1 * turnDirection);
                    //telemetry.addData("rightTicks", "" + FL.getCurrentPosition());
                    //telemetry.addData("leftTicks", "" + FR.getCurrentPosition());
                }
                break;
            case 7:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    ultrastate=ultra1.getUltrasonicLevel();
                    driveForever(0.6);
                    if(ultrastate < 10);
                    {
                        setLeftPower(0);
                        setRightPower(0);
                        //v_state++;
                    }
                    telemetry.addData("sav1", "Sensor1: " + ultrastate);
                }
                break;
            case 8:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    climberDumper.setPosition(0);
                    if (climberDumper.getPosition()< 0.2) {
                        v_state++;
                    }
                }
                break;
            case 9:
                pause (100);
                break;
            case 10:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    climberDumper.setPosition(.92);
                    if (climberDumper.getPosition()> 0.9) {
                        v_state++;
                    }
                }
            case 11:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                   drive(-1000, 0.5);

                }
            case 12:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    turn(45, -1);
                }
            case 13:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    drive(-500, 0.5);
                }
            case 14:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    turn(90, -1);
                }

            case 15:
                if(encoders_have_reset || have_drive_encoders_reset()) {
                    encoders_have_reset=true;
                    drive(-3000, 0.5);
                }
            default:
        }
        //telemetry.addData("Text", "State: " + v_state + " " + have_drive_encoders_reset()+ " " + encoders_have_reset);
        
        telemetry.addData("Text", "State: " + v_state + " " + FR.getCurrentPosition() + " " + BR.getCurrentPosition()+ " " + FL.getCurrentPosition() + " " + BL.getCurrentPosition());
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
            collector.setPower(1);
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
        if(hasLeftReached(degrees*TURNRATIO)||hasRightReached(degrees * TURNRATIO)) {
                setLeftPower(0);
                setRightPower(0);
                reset_drive_encoders();
                v_state++;
            }
            else{
                run_using_encoders();
            setLeftPower(power);
            setRightPower(-power);
            }
        }

}
