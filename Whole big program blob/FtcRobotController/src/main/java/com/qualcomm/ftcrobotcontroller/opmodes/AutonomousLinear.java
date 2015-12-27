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

public abstract class AutonomousLinear extends LinearOpMode {
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DeviceInterfaceModule cdim;
    GyroSensor gyroSensor;
    ColorSensor colorSensor;
    DcMotor FR;
    int FRold, BRold, FLold, BLold;
    DcMotor collector;
    Servo climberDumper;
    Servo sideArmL;
    Servo lock;
    Servo sideArmR;
    boolean turnComplete = false;
    Servo debDumper;
    OpticalDistanceSensor odm;
    Servo door;
    final int LED_CHANNEL = 5;
    UltrasonicSensor ultra1;
    UltrasonicSensor ultra2;
    int lastgyro;
    double turnChange = 1;
    int turnDirection = 1;
    private final double TURNRATIO = 18.3;
    private boolean didEncodersReset = false;


    public void runOpMode(int turnDirectionInput, double turnChangeInput) throws InterruptedException {
        turnChange = turnChangeInput;
        turnDirection = turnDirectionInput;
        getRobotConfig();
        run_using_encoders();
        reset_drive_encoders();

        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        gyroSensor.calibrate();
        lock.setPosition(0);
        climberDumper.setPosition(.92);
        sideArmL.setPosition(1);
        sideArmR.setPosition(0);
        sleep(5000);

        waitForStart(); //everything before this happens when you press init

        //  collector.setPower(1);
//        turn(90);
//        sleep(200);
//        turn(-90);
//        sleep(200);
//        turn(50);
//        sleep(200);
//        turn(-50);

        drive(2000, 1);
        sleep(200);

        turn(40);
        sleep(200);
        reset_drive_encoders();

        drive(4900, 1);
        sleep(500);

        turn(40);
        sleep(200);

        driveUntilUltra(25, 0.2);
        sleep(200);

        squareUp();
        sleep(200);

        turn(-80);
        sleep(200);

        while (readFixedODM(odm) < 900) {
            driveForever(0.2);
            waitOneFullHardwareCycle();
        }
        stopMotors();
        sleep(200);
        turn(85);
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

        drive(50  , 0.5, true);
        sleep(200);

        telemetry.addData("Red, Blue", " " + colorSensor.blue() + " " + colorSensor.red());
        sleep(100);
        waitOneFullHardwareCycle();

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

        squareUp();
        sleep(200);

        drive(100, -0.5);
        sleep(200);

        turn(-80);
        sleep(200);

        drive(500, -0.5);
        sleep(200);

        turn(45);
        sleep(200);

        drive(2400, -0.5);
        sleep(200);

        turn(-80);
        sleep(200);


        if (turnDirectionInput == 1)
            sideArmL.setPosition(0);
        else
            sideArmR.setPosition(1);
        sleep(200);

        drive(5000, -1);
        sleep(200);

        lock.setPosition(0.6);

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
        sideArmL = hardwareMap.servo.get("sideArmL");
        sideArmR = hardwareMap.servo.get("sideArmR");
        lock = hardwareMap.servo.get("lock");
        gyroSensor = hardwareMap.gyroSensor.get("G1");
        climberDumper = hardwareMap.servo.get("climberdumper");
        debDumper = hardwareMap.servo.get("debDumper");
        door = hardwareMap.servo.get("door");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        colorSensor = hardwareMap.colorSensor.get("cs1");
        collector = hardwareMap.dcMotor.get("colmot");
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        ultra1 = hardwareMap.ultrasonicSensor.get("ultra1");
        ultra2 = hardwareMap.ultrasonicSensor.get("ultra2");
        odm = hardwareMap.opticalDistanceSensor.get("odm1");
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
        double speed = 0.5;
        telemetry.addData("heading ", "" + heading());
        if (degrees < 0)
            degrees += 360;
        if (Math.abs(degrees - heading())<20){
            speed = 0.1;
        }

        if (degrees > 180) {
            do {
                telemetry.addData("heading ", "" + heading());
                run_using_encoders();


                FR.setPower(-speed);
                BR.setPower(-speed);
                FL.setPower(-speed);
                BL.setPower(-speed);
                waitOneFullHardwareCycle();
            } while (heading() > degrees || heading() < 20);
            run_using_encoders();
            stopMotors();

        } else {
            do {
                telemetry.addData("heading ", "" + heading());
                run_using_encoders();
                FR.setPower(speed);
                BR.setPower(speed);
                FL.setPower(speed);
                BL.setPower(speed);
                waitOneFullHardwareCycle();
            } while (degrees > heading() || heading() > 340);
            run_using_encoders();
            stopMotors();
        }
        run_using_encoders();
        stopMotors();
        telemetry.addData("Done", "");

    }

    int heading() {
        int head;
        head = gyroSensor.getHeading() - lastgyro;
        if (head < 0)
            head += 360;
        return (head);
    }

    double readFixedUltra(UltrasonicSensor sensor) {
        double val = 0;
        for (int i = 0; i < 10; i++) {
            val += sensor.getUltrasonicLevel();
        }
        val /= 10;
        return val;
    }

    int readFixedODM(OpticalDistanceSensor odm) {
        int val = 0;
        for (int i = 0; i < 10; i++) {
            val += odm.getLightDetectedRaw();
        }
        val /= 10;
        return val;
    }

    void squareUp() throws InterruptedException {

        while (Math.abs(readFixedUltra(ultra1) - readFixedUltra(ultra2)) != 0) {
            telemetry.addData("ultra1", readFixedUltra((ultra1)));
            telemetry.addData("ultra2", readFixedUltra((ultra2)));
            setLeftPower((readFixedUltra(ultra1) - readFixedUltra(ultra2)) / 50);
            setRightPower((readFixedUltra(ultra2) - readFixedUltra(ultra1)) / 50);
            waitOneFullHardwareCycle();

        }
        stopMotors();
    }

    void stopMotors() throws InterruptedException {
        while (FR.isBusy() == true ||
                BR.isBusy() == true ||
                FL.isBusy() == true ||
                BL.isBusy() == true) {
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

    boolean blocked() {
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