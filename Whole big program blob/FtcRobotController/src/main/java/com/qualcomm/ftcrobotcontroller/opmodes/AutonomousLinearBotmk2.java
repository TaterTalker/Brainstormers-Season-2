package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

public abstract class AutonomousLinearBotmk2 extends LinearOpMode {
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    GyroSensor gyroSensor;
    ColorSensor colorSensor;
    ColorSensor colorSensor2;
    DcMotor FR;
    int FRold, BRold, FLold, BLold;
    DcMotor collector;
    Servo climberDumper;
    //Servo sideArmL;
    //Servo lock;
    //Servo sideArmR;
    boolean turnComplete = false;
    Servo debDumper;
    OpticalDistanceSensor odm;
    final int LED_CHANNEL = 5;
    UltrasonicSensor ultra1;
    UltrasonicSensor ultra2;
    int lastgyro;
    double turnChange = 1;
    int turnDirection = 1;
    private final double TURNRATIO = 18.3;
    private boolean didEncodersReset = false;
    Servo someServo1; //I have no idea what these are supposed to do --> bad name
    Servo someServo2;


    public void runOpMode(int turnDirectionInput, double turnChangeInput) throws InterruptedException {
        turnChange = turnChangeInput;
        turnDirection = turnDirectionInput;
        getRobotConfig();
        run_using_encoders();
        reset_drive_encoders();

        gyroSensor.calibrate();
        //lock.setPosition(0);
        climberDumper.setPosition(.92);
        someServo1.setPosition(0.5);
        someServo2.setPosition(0.5);
        debDumper.setPosition((turnDirection+1)/2);
        //sideArmL.setPosition(1);
        //sideArmR.setPosition(0);
        sleep(5000);

        waitForStart(); //everything before this happens when you press init

        collector.setPower(-1);
//        turn(90);
//        sleep(200);
//        turn(-90);
//        sleep(200);
//        turn(50);
//        sleep(200);
//        turn(-50);
        final boolean SLEEP = false;
        if (SLEEP) {

        sleep(5000);
        }

        drive(2000, 1);
        sleep(200);

        turn(30);
        sleep(200);
        reset_drive_encoders();

        drive(4700, 1);
        sleep(500);

        turn(30);
        turn(30);//two turn because 60 degree turn messes up
        sleep(200);

        driveUntilUltra(20, 0.2);
        sleep(200);

        squareUp();
        sleep(200);

        turn(-45);
        turn(-45);
        sleep(200);

        while (colorSensor2.alpha() < 15) {
            driveForever(0.4);
            waitOneFullHardwareCycle();
        }
        stopMotors();
        sleep(200);
        turn(90);
        sleep(200);

        squareUp();
        collector.setPower(0);
        sleep(200);

        driveUntilUltra(10, 0.2);
        sleep(200);
        turn(17* turnDirection);

        stopMotors();
        sleep(200);
        climberDumper.setPosition(0);
        sleep(1000);
        climberDumper.setPosition(1);
        sleep(200);

        turn(-17*turnDirection);

        telemetry.addData("Red, Blue", " " + colorSensor.blue() + " " + colorSensor.red());
        sleep(100);
        waitOneFullHardwareCycle();
    /* BEACON
        if (colorSensor.blue() > colorSensor.red() && Math.abs(colorSensor.blue() - colorSensor.red()) > 20) {
            turn(5);
            sleep(200);
            drive(200, 0.25, true);
            sleep(200);
            drive(200, -0.25, true);
            sleep(200);
            turn(-5);
            sleep(200);
        } else if (Math.abs(colorSensor.blue() - colorSensor.red()) > 20) {
            turn(-5);
            sleep(200);
            drive(200, 0.25, true);
            sleep(200);
            drive(200, -0.25, true);
            sleep(200);
            turn(5);
            sleep(200);
        }
        */
        drive(-200,0.2);
        squareUp();
        sleep(200);

        drive(100, -0.5);
        sleep(200);

        turn(110);
        sleep(200);

        drive(2800, 1);
        sleep(200);

        turn(105);
        sleep(200);

        drive(1000, -0.5);


        if (turnDirectionInput == 1){}
            //sideArmL.setPosition(0);
        else{}
            //sideArmR.setPosition(1);

        //lock.setPosition(0.6);

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


        didEncodersReset = false;
    }

    void run_using_encoders() {

        FL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        FR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    boolean hasLeftReached(double leftd) {

        return (Math.abs(FLposition()) > leftd) &&
                (Math.abs(BLposition()) > leftd);
    }

    boolean hasRightReached(double rightd) {

        return (Math.abs(FRposition()) > rightd) &&
                (Math.abs(BRposition()) > rightd);
    }

    void setLeftPower(double power) {
        power = clip(power, -1, 1);
        run_using_encoders();
        FL.setPower(-power);
        BL.setPower(-power);
    }

    void setRightPower(double power) {
        power = clip(power, -1, 1);
        run_using_encoders();
        FR.setPower(power);
        BR.setPower(power);
    }

    double clip(double variable, double min, double max) {
        if (variable < min)
            variable = min;

        if (variable > max)
            variable = max;

        return variable;
    }

    void getRobotConfig() {
        //sideArmL = hardwareMap.servo.get("sideArmL");
        //sideArmR = hardwareMap.servo.get("sideArmR");
        //lock = hardwareMap.servo.get("lock");
        gyroSensor = hardwareMap.gyroSensor.get("G1");
        climberDumper = hardwareMap.servo.get("climberDumper");
        debDumper = hardwareMap.servo.get("dumper");
        colorSensor = hardwareMap.colorSensor.get("cs1");
        colorSensor2 = hardwareMap.colorSensor.get("cs2");
        collector = hardwareMap.dcMotor.get("collect");
        FR = hardwareMap.dcMotor.get("fr");
        FL = hardwareMap.dcMotor.get("fl");
        BR = hardwareMap.dcMotor.get("br");
        BL = hardwareMap.dcMotor.get("bl");
        ultra1 = hardwareMap.ultrasonicSensor.get("ultraL");
        ultra2 = hardwareMap.ultrasonicSensor.get("ultraR");
        odm = hardwareMap.opticalDistanceSensor.get("odm");
        someServo1 = hardwareMap.servo.get("armAngle1");
        someServo2 = hardwareMap.servo.get("armAngle2");
    }

    void resetGyro() throws InterruptedException {
        while (heading() != 0) {
            lastgyro = gyroSensor.getHeading();
            waitOneFullHardwareCycle();
        }
    }

    void turn(int degrees) throws InterruptedException {
        resetGyro();
        degrees *= turnDirection;
        int dir;
        dir=heading();
        while(Math.abs(degrees-dir)>1) {
            telemetry.addData("difference", " " + Math.abs(degrees-dir));
            dir = heading();
            if (dir > 180)
                dir -= 360;

            double power=(degrees-dir)/40;
            telemetry.addData("initial power", " " + power);
            power=clip(power,-0.15,0.15);
            if(Math.abs(power)<0.05){
                if((degrees-dir)>0)
                    power= 0.07;
                else
                    power= -0.07;
            }

            telemetry.addData("power", " " + power);

            setLeftPower(-power);
            setRightPower(power);
            waitOneFullHardwareCycle();
        }
        stopMotors();
    }

    int heading() {
        int head;
        head = gyroSensor.getHeading() - lastgyro;
        if (head < 0)
            head += 360;
        return (head);
    }

    double readFixedUltra(UltrasonicSensor sensor) throws InterruptedException {
        double val = 0;
        double maxVal = 0;
        double minVal = 255;
        double tmpVal;
        for (int i = 0; i < 4; i++) {
            do {
                tmpVal = sensor.getUltrasonicLevel();
            }while(tmpVal==0);
            if (tmpVal < minVal)
                minVal = tmpVal;
            if (tmpVal > maxVal)
                maxVal = tmpVal;
            val += tmpVal;
            waitOneFullHardwareCycle();
        }
        val -= (minVal + maxVal);
        val /= 2;
        return val;
    }


    int readFixedODM(OpticalDistanceSensor odm) throws InterruptedException {
        int val = 0;
        for (int i = 0; i < 2; i++) {
            val += odm.getLightDetectedRaw();
            waitOneFullHardwareCycle();
        }
        val /= 2;
        return val;
    }


    void squareUp() throws InterruptedException {

        while (Math.abs(readFixedUltra(ultra1) - readFixedUltra(ultra2)) > 1) {
            telemetry.addData("ultra1", readFixedUltra((ultra1)));
            telemetry.addData("ultra2", readFixedUltra((ultra2)));
            double power = (ultra1.getUltrasonicLevel() - ultra2.getUltrasonicLevel()) / 70;
            power = clip(power, -0.15,0.15);
            setLeftPower(-power);
            setRightPower(power);
            waitOneFullHardwareCycle();
        }
        stopMotors();
    }

    void stopMotors() throws InterruptedException {
        while (FR.isBusy() || BR.isBusy() || FL.isBusy() || BL.isBusy()) {
            run_using_encoders();
            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
            waitOneFullHardwareCycle();
        }
    }

    void driveUntilUltra(int target, double speed) throws InterruptedException {
        while (readFixedUltra(ultra1) > target || readFixedUltra(ultra1) < 1) {
            driveForever(speed);
            waitOneFullHardwareCycle();
        }
        stopMotors();
    }

    void drive(float distance, double speed, boolean noAvoidance) throws InterruptedException {
        resetEncoderDelta();
        resetGyro();
        // Start the drive wheel motors at full power
        while (!hasLeftReached(distance) && !hasRightReached(distance)) {
            double currSpeed = speed;
            // telemetry.addData("encoder values", "right:" + FR.getCurrentPosition() + " left:" + FL.getCurrentPosition());
            double turnheading = heading();
            if (turnheading > 180)
                turnheading -= 360;
            turnheading /= 15;

            if (Math.abs(turnheading) > 1)
                currSpeed = clip(currSpeed, -0.7, 0.7);
            else if (turnheading != 0)
                currSpeed = clip(currSpeed, -0.9, 0.9);

            telemetry.addData("heading ", "" + heading());
            run_using_encoders();
            setLeftPower(currSpeed + turnheading);
            setRightPower(currSpeed - turnheading);
            waitOneFullHardwareCycle();
        }
        stopMotors();
        reset_drive_encoders();
    }

    void drive(float distance, double speed) throws InterruptedException {
        resetEncoderDelta();
        resetGyro();
        // Start the drive wheel motors at full power
        while (!hasLeftReached(distance) && !hasRightReached(distance)) {
            double activeSpeed = speed;
            // telemetry.addData("encoder values", "right:" + FR.getCurrentPosition() + " left:" + FL.getCurrentPosition());
            double turnheading = heading();
            if (turnheading > 180)
                turnheading -= 360;
            turnheading /= 15;

            if (blocked() && speed > 0)
                activeSpeed = 0;

            else if (Math.abs(turnheading) > 1)
                activeSpeed = clip(activeSpeed, -0.7, 0.7);
            else if (turnheading != 0)
                activeSpeed = clip(activeSpeed, -0.9, 0.9);

            telemetry.addData("heading ", "" + heading());
            run_using_encoders();
            setLeftPower(activeSpeed + turnheading);
            setRightPower(activeSpeed - turnheading);
            waitOneFullHardwareCycle();
        }
        stopMotors();
        reset_drive_encoders();
    }

    boolean blocked() throws InterruptedException {
        return (readFixedUltra(ultra1) < 20 || readFixedUltra(ultra2) < 20);
    }

    int FRposition() {
        return (FR.getCurrentPosition() - FRold);
    }

    int BRposition() {
        return (BR.getCurrentPosition() - BRold);
    }

    int FLposition() {
        return (FL.getCurrentPosition() - FLold);
    }

    int BLposition() {
        return (BL.getCurrentPosition() - BLold);
    }

    void resetEncoderDelta() {
        FRold = FR.getCurrentPosition();
        BRold = BR.getCurrentPosition();
        FLold = FL.getCurrentPosition();
        BLold = BL.getCurrentPosition();
    }

}