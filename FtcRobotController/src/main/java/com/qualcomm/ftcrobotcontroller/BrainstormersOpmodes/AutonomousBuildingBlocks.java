package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by August on 2/8/2016.
 */
public abstract class AutonomousBuildingBlocks extends LinearOpMode {

   //Controllers
    AdafruitIMUmethods adaFruitGyro;
    Cameracontroller cameraController;

    //Driving Motors
    DcMotor fl;
    DcMotor fr;
    DcMotor br;
    DcMotor bl;
    DcMotor collector;

    int fRold;
    int bRold;
    int fLold;
    int bLold;
    int delay=0;

    ColorSensor colorSensor;
    UltrasonicSensor ultra1;
    UltrasonicSensor ultra2;

    //Servo
    Servo beaconR;
    Servo beaconL;
    Servo climberDumper;
    Servo debDumper;
    Servo sideArmL;
    Servo sideArmR;
    Servo doorR;
    Servo doorL;

    //Variables
    int turnDirection = 1;
    private boolean didEncodersReset = false;
    Servo armAngle1; //I have no idea what these are supposed to do --> bad name
    Servo armAngle2;


    public AutonomousBuildingBlocks (){
        super();
        adaFruitGyro = new AdafruitIMUmethods(this);
        cameraController = new Cameracontroller(this);

    }

    //Our Main Function that Controls the Robots Actions\

    /**
     * maps everything to the hardware
     */
    //Configures the Robots Motors and Sensors.
    void getRobotConfig() {

        //Sensors
        climberDumper = hardwareMap.servo.get("climberDumper");
        colorSensor = hardwareMap.colorSensor.get("cs2");
        collector = hardwareMap.dcMotor.get("collect");
        ultra1 = hardwareMap.ultrasonicSensor.get("ultraL");
        ultra2 = hardwareMap.ultrasonicSensor.get("ultraR");
        debDumper = hardwareMap.servo.get("dumper");
        sideArmL = hardwareMap.servo.get("sideArmL");
        sideArmR = hardwareMap.servo.get("sideArmR");
        beaconR = hardwareMap.servo.get("beacon right");
        beaconL = hardwareMap.servo.get("beacon left");

        //Motors
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
        armAngle1 = hardwareMap.servo.get("armAngle1");
        armAngle2 = hardwareMap.servo.get("armAngle2");
        doorR = hardwareMap.servo.get("doorR");
        doorL = hardwareMap.servo.get("doorL");
    }

    /**
     * sets all drive encoders to 0
     */
    //Resets the Drive Encoders.
    void resetDriveEncoders() {

        fl.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        fr.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        bl.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        br.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        didEncodersReset = false;
    }

    /**
     * sets all drive encoders to run using encoders mode
     */
    //Run using Encoders.
    void runUsingEncoders() {

        fl.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        fr.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        bl.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        br.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    /**
     * determines if the left side has reached the target disance
     *
     * @param leftDistance target distance
     * @return if the encoders are greater than or equal to the target distance
     */
    //Has the left side reached a certain encoder value.
    /**
     * sets the left motors to a specific power, keeping in mind the acceptable limits
     * sets the left motors to a specific power, keeping in mind the acceptable limits
     *
     * @param power target power
     */
    //Set the motors power.
    void setLeftPower(double power) {
        power = clip(power, -1, 1);
        runUsingEncoders();
        fl.setPower(power);
        bl.setPower(power);
    }

    /**
     * sets the right motors to a specific power, keeping in mind the acceptable limits
     *
     * @param power target power
     */
    //Set the motors power.
    void setRightPower(double power) {
        power = clip(power, -1, 1); //because david is trash at building robots and it turns by itself we need to slow this side down, you know it would be so much more effective if you did it in hardware you ass - August
        runUsingEncoders();
        fr.setPower(-power);
        br.setPower(-power);
    }

    /**
     * takes a variable and returns the variable
     * if the variable is outside the one of the bounds, instead it returns that bound
     *
     * @param variable the variable to be clipped
     * @param MIN      the minimum exeptable value
     * @param MAX      the maximum exeptable value
     * @return returns the variable, exept it is inside the exeptable vlues
     */
    //Clip the values.
    double clip(double variable, double MIN, double MAX) {

        if (variable < MIN) {
            variable = MIN;
        }
        if (variable > MAX) {
            variable = MAX;
        }

        return variable;
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
            if (tmpVal < minVal) {
                minVal = tmpVal;
            }
            if (tmpVal > maxVal) {
                maxVal = tmpVal;
            }
            val += tmpVal;
            waitOneFullHardwareCycle();
        }
        val -= (minVal + maxVal);
        val /= 2;
        return val;
    }

    /**
     * stops all motors and verifies that they are stopped
     *
     * @throws InterruptedException
     */
    void stopMotors() throws InterruptedException {
        int iterations = 0;
        while ((fr.isBusy() || br.isBusy() || fl.isBusy() || bl.isBusy()) && iterations < 10) {
            iterations++;
            telemetry.addData("motor status", fr.isBusy() + " " + br.isBusy() + " " + fl.isBusy() + " " + bl.isBusy());
            runUsingEncoders();
            br.setPower(0);
            bl.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
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
        final int STOPDIST = 20;

        return (readFixedUltra(ultra1) < STOPDIST || readFixedUltra(ultra2) < STOPDIST);
    }

    /**
     * gets adjusted position of the front right motor
     * reset to 0 using {@link #resetEncoderDelta()}
     *
     * @return the adjusted value
     */
    int frPosition() {
        return (fr.getCurrentPosition() - fRold);
    }

    /**
     * gets adjusted position of the back right motor
     * reset to 0 using {@link #resetEncoderDelta()}
     *
     * @return the adjusted value
     */
    int brPosition() {
        return (br.getCurrentPosition() - bRold);
    }

    /**
     * gets adjusted position of the front left motor
     * reset to 0 using {@link #resetEncoderDelta()}
     *
     * @return the adjusted value
     */
    int fLPosition() {
        return (fLold - fl.getCurrentPosition());
    }

    /**
     * gets adjusted position of the back left motor
     * reset to 0 using {@link #resetEncoderDelta()}
     *
     * @return the adjusted value
     */
    int blPosition() {
        return (bLold - bl.getCurrentPosition());
    }

    /**
     * resets {@link #frPosition()} {@link #brPosition()} {@link #fLPosition()} {@link #blPosition()}
     */
    void resetEncoderDelta() {
        fRold = fr.getCurrentPosition();
        bRold = br.getCurrentPosition();
        fLold = fl.getCurrentPosition();
        bLold = bl.getCurrentPosition();
    }


}
