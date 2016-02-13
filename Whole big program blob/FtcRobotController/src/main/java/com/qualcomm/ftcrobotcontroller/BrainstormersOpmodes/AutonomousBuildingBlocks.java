package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by August on 2/8/2016.
 */
public abstract class AutonomousBuildingBlocks extends CameraOp {


    //Driving Motors
    DcMotor FL;
    DcMotor FR;
    DcMotor BR;
    DcMotor BL;

    int FRold, BRold, FLold, BLold;

    //Sensors
    Servo beacon;
    GyroSensor gyroSensor;
    int lastgyro;
    ColorSensor colorSensor2;
    DcMotor collector;
    UltrasonicSensor ultra1;
    UltrasonicSensor ultra2;


    //Servos
    Servo climberDumperB;
    //Servo climberDumperR;
    Servo sideArmL;
    Servo debDumper;
    Servo sideArmR;
    Servo doorR;
    Servo doorL;

    //Variables
    int turnDirection = 1;
    private boolean didEncodersReset = false;
    Servo armAngle1; //I have no idea what these are supposed to do --> bad name
    Servo armAngle2;

    //Our Main Function that Controls the Robots Actions\

    /**
     * maps everything to the hardware
     */
    //Configures the Robots Motors and Sensors.
    void getRobotConfig() {
        //sideArmL = hardwareMap.servo.get("sideArmL");
        //sideArmR = hardwareMap.servo.get("sideArmR");
        //lock = hardwareMap.servo.get("lock");

        //Sensors
        gyroSensor = hardwareMap.gyroSensor.get("G1");
        climberDumperB = hardwareMap.servo.get("climberDumper");
        // colorSensor = hardwareMap.colorSensor.get("cs1");
        colorSensor2 = hardwareMap.colorSensor.get("cs2");
        collector = hardwareMap.dcMotor.get("collect");
        ultra1 = hardwareMap.ultrasonicSensor.get("ultraL");
        ultra2 = hardwareMap.ultrasonicSensor.get("ultraR");
        //   odm = hardwareMap.opticalDistanceSensor.get("odm");
        debDumper = hardwareMap.servo.get("dumper");
        sideArmL = hardwareMap.servo.get("sideArmL");
        sideArmR = hardwareMap.servo.get("sideArmR");
        beacon = hardwareMap.servo.get("beacon");

        //Motors
        FR = hardwareMap.dcMotor.get("fr");
        FL = hardwareMap.dcMotor.get("fl");
        BR = hardwareMap.dcMotor.get("br");
        BL = hardwareMap.dcMotor.get("bl");
        armAngle1 = hardwareMap.servo.get("armAngle1");
        armAngle2 = hardwareMap.servo.get("armAngle2");
        doorR = hardwareMap.servo.get("doorR");
        doorL = hardwareMap.servo.get("doorL");
    }

    /**
     * sets all motors to a specific speed
     *
     * @param speed target speed
     */
    void driveForever(double speed) {
        // Start the drive wheel motors at full power
        run_using_encoders();
        setLeftPower(speed);
        setRightPower(speed);
    }

    void pivotleft(double distance) throws InterruptedException {
        run_using_encoders();
        while (!hasLeftReached(distance)) {
            setLeftPower(0.5);
            waitOneFullHardwareCycle();
        }
        stopMotors();
    }

    /**
     * sets all drive encoders to 0
     */
    //Resets the Drive Encoders.
    void reset_drive_encoders() {

        FL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        FR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        BL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        BR.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        didEncodersReset = false;
    }

    /**
     * sets all drive encoders to run using encoders mode
     */
    //Run using Encoders.
    void run_using_encoders() {

        FL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        FR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    /**
     * determines if the left side has reached the target disance
     *
     * @param leftd target distance
     * @return if the encoders are greater than or equal to the target distance
     */
    //Has the left side reached a certain encoder value.
    boolean hasLeftReached(double leftd) {

        return (Math.abs(FLposition()) > leftd) && (Math.abs(BLposition()) > leftd);
    }

    /**
     * determines if the left side has reached the target disance
     *
     * @param rightd target distance
     * @return if the encoders are greater than or equal to the target distance
     */
    //Has the right side reached a certain encoder value.
    boolean hasRightReached(double rightd) {

        return (Math.abs(FRposition()) > rightd) && (Math.abs(BRposition()) > rightd);
    }
    /**
     * sets the left motors to a specific power, keeping in mind the acceptable limits
     *
     * @param power target power
     */
    //Set the motors power.
    void setLeftPower(double power) {

        power = clip(power, -1, 1);
        run_using_encoders();
        FL.setPower(-power);
        BL.setPower(-power);
    }

    /**
     * sets the right motors to a specific power, keeping in mind the acceptable limits
     *
     * @param power target power
     */
    //Set the motors power.
    void setRightPower(double power) {

        power = clip(power, -0.95, 0.95); //because david is trash at building robots and it turns by itself we need to slow this side down, you know it would be so much more effective if you did it in hardware you ass - August
        run_using_encoders();
        FR.setPower(power);
        BR.setPower(power);
    }

    /**
     * takes a variable and returns the variable
     * if the variable is outside the one of the bounds, instead it returns that bound
     *
     * @param variable the variable to be clipped
     * @param min      the minimum exeptable value
     * @param max      the maximum exeptable value
     * @return returns the variable, exept it is inside the exeptable vlues
     */
    //Clip the values.
    double clip(double variable, double min, double max) {

        if (variable < min) variable = min;
        if (variable > max) variable = max;

        return variable;
    }

    /**
     * resets the gyro delta
     *
     * @throws InterruptedException
     * @see #heading()
     */
    //Resets the gyro based on the old heading.
    void resetGyro() throws InterruptedException {

        while (gyroSensor.getHeading() - lastgyro != 0) {
            lastgyro = gyroSensor.getHeading();
            waitOneFullHardwareCycle();
        }
    }
    /**
     * gets an adjusted headier that can be reset
     *
     * @return the adjusted heading
     */
    int heading() {
        int head;
        head = gyroSensor.getHeading() - lastgyro;
        if (head < 0)
            head += 360;
        return (head);
    }

    /**
     * takes multiple ultrasonic readins and throws out anomalies
     * to get a more precise ul;trasonic value
     *
     * @param sensor the sensor to be read
     * @return the more accurate ultrasonic value
     * @throws InterruptedException
     */
    //
    double readFixedUltra(UltrasonicSensor sensor) throws InterruptedException {

        double val = 0;
        double maxVal = 0;
        double minVal = 255;
        double tmpVal;

        for (int i = 0; i < 4; i++) {
            do {
                tmpVal = sensor.getUltrasonicLevel();
                waitOneFullHardwareCycle();
            } while (tmpVal == 0);
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

    /**
     * uses two ultrasonic sensors to square up against a wall
     *
     * @throws InterruptedException
     * @see #readFixedUltra(UltrasonicSensor)
     */
    void squareUp() throws InterruptedException {

        while (Math.abs(readFixedUltra(ultra1) - readFixedUltra(ultra2)) > 1) {
            telemetry.addData("ultra1", readFixedUltra((ultra1)));
            telemetry.addData("ultra2", readFixedUltra((ultra2)));
            double power = (ultra1.getUltrasonicLevel() - ultra2.getUltrasonicLevel()) / 70;
            power = clip(power, -0.15, 0.15);
            setLeftPower(-power);
            setRightPower(power);
            waitOneFullHardwareCycle();
        }
        stopMotors();
    }

    /**
     * stops all motors and verifies that they are stopped
     *
     * @throws InterruptedException
     */
    void stopMotors() throws InterruptedException {
        int iterations=0;
        while ((FR.isBusy() || BR.isBusy() || FL.isBusy() || BL.isBusy())&&iterations<10) {
            iterations++;
            telemetry.addData("motor status", FR.isBusy() + " " + BR.isBusy() + " " + FL.isBusy() + " " + BL.isBusy());
            run_using_encoders();
            BR.setPower(0);
            BL.setPower(0);
            FL.setPower(0);
            FR.setPower(0);
            sleep(1);
        }
        telemetry.addData("stopping", "complete");
    }
    /**
     * determines whether the robot is blocked
     *
     * @return if something is detected within 20 cm, TRUE
     * @throws InterruptedException
     * @see #readFixedUltra(UltrasonicSensor)
     */
    boolean blocked() throws InterruptedException {
        return (readFixedUltra(ultra1) < 20 || readFixedUltra(ultra2) < 20);
    }

    /**
     * gets adjusted position of the front right motor
     * reset to 0 using {@link #resetEncoderDelta()}
     *
     * @return the adjusted value
     */
    int FRposition() {
        return (FR.getCurrentPosition() - FRold);
    }

    /**
     * gets adjusted position of the back right motor
     * reset to 0 using {@link #resetEncoderDelta()}
     *
     * @return the adjusted value
     */
    int BRposition() {
        return (BR.getCurrentPosition() - BRold);
    }

    /**
     * gets adjusted position of the front left motor
     * reset to 0 using {@link #resetEncoderDelta()}
     *
     * @return the adjusted value
     */
    int FLposition() {
        return (FLold-FL.getCurrentPosition());
    }

    /**
     * gets adjusted position of the back left motor
     * reset to 0 using {@link #resetEncoderDelta()}
     *
     * @return the adjusted value
     */
    int BLposition() {
        return (BLold-BL.getCurrentPosition());
    }

    /**
     * resets {@link #FRposition()} {@link #BRposition()} {@link #FLposition()} {@link #BLposition()}
     */
    void resetEncoderDelta() {
        FRold = FR.getCurrentPosition();
        BRold = BR.getCurrentPosition();
        FLold = FL.getCurrentPosition();
        BLold = BL.getCurrentPosition();
    }

}
