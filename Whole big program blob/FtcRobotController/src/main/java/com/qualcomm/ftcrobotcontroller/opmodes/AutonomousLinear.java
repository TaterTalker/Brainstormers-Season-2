package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

public abstract class AutonomousLinear extends LinearOpMode {
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
    double turnChange=1;
    int turnDirection=1;
    private final double TURNRATIO = 18.3;
    private int loopCount = 0;
    private double ultrastate = 0;
    private boolean didEncodersReset=false;


    public void runOpMode(int turnDirectionInput, double turnChangeInput) throws InterruptedException {
        turnChange=turnChangeInput;
        turnDirection=turnDirectionInput;
        getRobotConfig();
        run_using_encoders();
        reset_drive_encoders();
        waitForStart();

        drive(2000, 1);
        sleep(100);
        turn(36, -0.5);
        sleep(1);

        drive(5800, 1);
        sleep(100);
        turn(60, -0.5);
        collector.setPower(0);
        sleep(1);

        while(ultrastate > 10 || ultrastate < 1) {
            ultrastate=ultra1.getUltrasonicLevel();
            driveForever(.2);
            telemetry.addData("Text", "Ultra: " + ultra1.getUltrasonicLevel());
        }
        setLeftPower(0);
        setRightPower(0);

        while(climberDumper.getPosition() > 0.2);

        sleep(200);
        climberDumper.setPosition(.92);
        while (climberDumper.getPosition() < 0.72)
            reset_drive_encoders();

        collector.setPower(1);
        drive(500, -0.5);
        sleep(100);

        turn(90, -0.5);
        drive(1600, 1);
        sleep(100);

        turn(30, -0.5);
        drive(1700, 1);
        sleep(100);

        if (turnDirection==-1)
            sideArmR.setPosition(1);
        else
            sideArmL.setPosition(0);
        turn(97, -0.5);
        driveForever(-1);
        sleep(300);
        setLeftPower(0);
        setRightPower(0);
        sideArmL.setPosition(1);
        sideArmR.setPosition(0);
    }


    void drive(float distance, double speed) {
        // Start the drive wheel motors at full power

        while (!hasLeftReached(distance) && !hasRightReached(distance)) {
            run_using_encoders();
            setLeftPower(speed);
            setRightPower(speed);
        }
        setLeftPower(0);
        setRightPower(0);
        reset_drive_encoders();
    }

    void drive(float distance, double speed, int withCollector) {
        // Start the drive wheel motors at full power
        if(encoders_have_reset()) {

            while (!hasLeftReached(distance) && !hasRightReached(distance)) {
                run_using_encoders();
                setLeftPower(speed);
                setRightPower(speed);
                collector.setPower(-1 * withCollector);
            }
            setLeftPower(0);
            setRightPower(0);
            collector.setPower(0);
            reset_drive_encoders();
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
        didEncodersReset=false;
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
        if(encoders_have_reset()) {
            if (power < 0)
                degrees = degrees * 1.05;
            run_using_encoders();
            while (!hasLeftReached(degrees * TURNRATIO * turnChange) && !hasRightReached(degrees * TURNRATIO * turnChange)) {
                setLeftPower(power * turnDirection);
                setRightPower(-power * turnDirection);
            }
            setLeftPower(0);
            setRightPower(0);
            reset_drive_encoders();
        }

    }

    void turnWithGyro(int degrees){
        /*if(encoders_have_reset()) {
        if(loopCount<400) {
            loopCount ++;
            return;
        }
        //telemetry.addData("heading: ", "" + gyroSensor.getHeading());
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
                else
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
            loopCount=0;
        }
        }
    }

    void resetGyro(){
        //telemetry.addData("heading: ", "" + gyroSensor.getHeading());
        gyroSensor.calibrate();
        while(gyroSensor.getHeading()!=0) {
        }*/
    }

    boolean encoders_have_reset() {
        if(didEncodersReset ||
                FL.getCurrentPosition() == 0 &&
                FR.getCurrentPosition() == 0 &&
                BL.getCurrentPosition() == 0 &&
                BR.getCurrentPosition() == 0)
            didEncodersReset=true;
            return true;
    }

    void getRobotConfig() {
        sideArmL = hardwareMap.servo.get("sideArmL");
        sideArmR = hardwareMap.servo.get("sideArmR");
        lock = hardwareMap.servo.get("lock");
        gyroSensor = hardwareMap.gyroSensor.get("G1");
        climberDumper = hardwareMap.servo.get("climberdumper");
        debDumper = hardwareMap.servo.get("debDumper");
        door = hardwareMap.servo.get("door");
        collector= hardwareMap.dcMotor.get ("colmot");
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        ultra1 = hardwareMap.ultrasonicSensor.get("ultra1");
    }

}

