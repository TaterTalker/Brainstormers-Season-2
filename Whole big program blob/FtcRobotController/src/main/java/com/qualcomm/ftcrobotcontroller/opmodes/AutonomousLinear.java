package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
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
    OpticalDistanceSensor odm;
    Servo door;
    UltrasonicSensor ultra1;
    UltrasonicSensor ultra2;
    int lastgyro;
    double turnChange=1;
    int turnDirection=1;
    private final double TURNRATIO = 18.3;
    private boolean didEncodersReset=false;


    public void runOpMode(int turnDirectionInput, double turnChangeInput) throws InterruptedException {
        turnChange=turnChangeInput;
        turnDirection=turnDirectionInput;
        getRobotConfig();
        run_using_encoders();
        reset_drive_encoders();
        gyroSensor.calibrate();
        sleep(5000);
        waitForStart(); //everything before this happens when you press init
        //collector.setPower(-1);

        driveStraightAvoidance(2000, 1);
        sleep(200);

        turnWithGyro(30);
        sleep(200);
        reset_drive_encoders();

        driveStraightAvoidance(5000, 1);
        sleep(200);

        turnWithGyro(70);
        sleep(200);

        driveUntilUltra(25, 0.2);
        sleep(200);

        squareUp();
        sleep(200);

        turnWithGyro(-90);
        sleep(200);

        while(readFixedODM(odm)<900) {
            driveForever(0.2);
        }
        stopMotors();
        sleep(200);
        turnWithGyro(90);
        sleep(200);

        squareUp();
        collector.setPower(0);
        sleep(200);

        driveUntilUltra(15, 0.2);
        sleep(200);

        stopMotors();
        sleep(200);
        climberDumper.setPosition(0);
        sleep(1000);
        climberDumper.setPosition(.92);
        sleep(200);

        driveStraight(1000, -1);
        sleep(200);

        turnWithGyro(-45);
        sleep(200);

        driveStraight(400, -1);
    }

    void drive(float distance, double speed) throws InterruptedException {
            reset_drive_encoders();
        // Start the drive wheel motors at full power
        while(!encoders_have_reset())
            sleep(1);

        while (!hasLeftReached(distance) && !hasRightReached(distance)) {
            telemetry.addData("encoder values", "right:" + FR.getCurrentPosition() + " left:" + FL.getCurrentPosition());
            run_using_encoders();
            setLeftPower(speed);
            setRightPower(speed);
        }
        stopMotors();
        reset_drive_encoders();
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
        run_using_encoders();
        FL.setPower(-power);
        BL.setPower(-power);
    }

    void setRightPower(double power) {
        power=clip(power,-1,1);
        run_using_encoders();
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
        ultra2 = hardwareMap.ultrasonicSensor.get("ultra2");
        odm = hardwareMap.opticalDistanceSensor.get("odm1");
    }

    void resetGyro() {
        while(heading()!=0) {
            lastgyro = gyroSensor.getHeading();
        }
    }

    void turnWithGyro(int degrees) throws InterruptedException {
        resetGyro();
        degrees*=turnDirection;
        telemetry.addData("heading ", "" + heading());
        if (degrees < 0)
            degrees += 370;


        else
            degrees-=10;

        if (degrees > 180) {
            do {
                telemetry.addData("heading ", "" + heading());
                run_using_encoders();
                FR.setPower(-0.5);
                BR.setPower(-0.5);
                FL.setPower(-0.5);
                BL.setPower(-0.5);
                sleep(5);
            } while (heading()>degrees || heading()<20);
            run_using_encoders();
            stopMotors();

        } else {
            do {
                telemetry.addData("heading ", "" + heading());
                run_using_encoders();
                FR.setPower(0.5);
                BR.setPower(0.5);
                FL.setPower(0.5);
                BL.setPower(0.5);
                sleep(5);
            } while (degrees > heading() || heading()>340);
            run_using_encoders();
            stopMotors();
        }
        run_using_encoders();
        stopMotors();
        telemetry.addData("Done","");

    }

    int heading(){
        int head;
        head=gyroSensor.getHeading()-lastgyro;
        if (head<0)
            head+=360;
        return (head);
    }

    double readFixedUltra(UltrasonicSensor sensor){
        double val = 0;
        for(int i=0;i<10;i++) {
            val+=sensor.getUltrasonicLevel();
        }
        val/=10;
        return val;
    }

    int readFixedODM(OpticalDistanceSensor odm){
        int val = 0;
        for(int i=0;i<10;i++) {
            val+=odm.getLightDetectedRaw();
        }
        val/=10;
        return val;
        }
    void squareUp() {

        while ( Math.abs(readFixedUltra(ultra1) - readFixedUltra(ultra2)) !=0 ) {
            telemetry.addData("ultra1", readFixedUltra((ultra1)));
            telemetry.addData("ultra2", readFixedUltra((ultra2)));
            setLeftPower((readFixedUltra(ultra1) - readFixedUltra(ultra2))/50);
            setRightPower((readFixedUltra(ultra2) - readFixedUltra(ultra1))/50);

        }
        stopMotors();
    }
    void stopMotors(){
        while(FR.isBusy()==true ||
                BR.isBusy()==true||
                FL.isBusy()==true||
                BL.isBusy()==true){
            run_using_encoders();
            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
        }
    }

    void driveUntilUltra(int target, double speed){
        while(readFixedUltra(ultra1) > target || readFixedUltra(ultra1) < 1) {
            driveForever(speed);
        }
        stopMotors();
    }

    void driveStraight(float distance, double speed) throws InterruptedException {
        reset_drive_encoders();
        resetGyro();
        // Start the drive wheel motors at full power
        while(!encoders_have_reset())
            sleep(1);

        while (!hasLeftReached(distance) && !hasRightReached(distance)) {
            double currSpeed=speed;
            telemetry.addData("encoder values", "right:" + FR.getCurrentPosition() + " left:" + FL.getCurrentPosition());
            double turnheading = heading();
            if(turnheading>180)
                turnheading-=360;
            turnheading/=15;

            if(Math.abs(turnheading)>1)
              currSpeed=   clip(currSpeed,-0.7,0.7);
            else if (turnheading!=0)
              currSpeed =  clip(currSpeed,-0.9,0.9);

            telemetry.addData("heading ", "" + heading());
            run_using_encoders();
            setLeftPower(currSpeed +turnheading);
            setRightPower(currSpeed - turnheading);
        }
        stopMotors();
        reset_drive_encoders();
    }

    void driveStraightAvoidance(float distance, double speed) throws InterruptedException {
        reset_drive_encoders();
        resetGyro();
        // Start the drive wheel motors at full power
        while(!encoders_have_reset())
            sleep(1);

        while (!hasLeftReached(distance) && !hasRightReached(distance)) {
            double activeSpeed=speed;
            telemetry.addData("encoder values", "right:" + FR.getCurrentPosition() + " left:" + FL.getCurrentPosition());
            double turnheading = heading();
            if(turnheading>180)
                turnheading-=360;
            turnheading/=15;

            if (blocked()&&speed>0)
                activeSpeed=0;

            else if(Math.abs(turnheading)>1)
                activeSpeed=clip(activeSpeed,-0.7,0.7);
            else if (turnheading!=0)
                activeSpeed=clip(activeSpeed,-0.9,0.9);

            telemetry.addData("heading ", "" + heading());
            run_using_encoders();
            setLeftPower(activeSpeed +turnheading);
            setRightPower(activeSpeed -turnheading);
        }
        stopMotors();
        reset_drive_encoders();
    }

    boolean blocked(){
        return (readFixedUltra(ultra1)<60||readFixedUltra(ultra1)<60);
    }
}

