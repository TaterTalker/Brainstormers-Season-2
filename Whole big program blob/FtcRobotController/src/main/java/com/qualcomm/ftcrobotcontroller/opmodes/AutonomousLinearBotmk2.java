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

    //Driving Motors
    DcMotor FL;
    DcMotor FR;
    DcMotor BR;
    DcMotor BL;
    int FRold, BRold, FLold, BLold;

    //Sensors
    GyroSensor gyroSensor;
    int lastgyro;
    ColorSensor colorSensor;
    ColorSensor colorSensor2;
    DcMotor collector;
    UltrasonicSensor ultra1;
    UltrasonicSensor ultra2;
    OpticalDistanceSensor odm;

    //Servos
    Servo climberDumper;
    Servo sideArmL;
    Servo debDumper;
    Servo sideArmR;

    //Variables
    int turnDirection = 1;
    private boolean didEncodersReset = false;
    Servo someServo1; //I have no idea what these are supposed to do --> bad name
    Servo someServo2;

    //Our Main Function that Controls the Robots Actions

    /**
     * run all other functions to perform autonomous
     * @param turnDirectionInput 1=blue -1=red
     * @throws InterruptedException
     */
    public void runOpMode(int turnDirectionInput) throws InterruptedException {

        //Adjusts turns based on team color.
        turnDirection = turnDirectionInput;

        //Map Motors and Sensors.
        getRobotConfig();

        //Configure and Reset.
        run_using_encoders();
        reset_drive_encoders();
        gyroSensor.calibrate();
        climberDumper.setPosition(0);
        someServo1.setPosition(0.5);
        someServo2.setPosition(0.5);
        debDumper.setPosition((turnDirection + 1) / 2);

        sleep(5000);

        waitForStart(); //everything before this happens when you press init

        collector.setPower(-1);
        final boolean SLEEP = false;

        //pivotleft(200);
        if (SLEEP) sleep(5000);

        drive(2000, 1);
        sleep(200);

        turn(30);
        sleep(200);

        drive(4400, 1);
        sleep(500);

        turn(60);
        collector.setPower(1);
        sleep(200);

        squareUp();
        sleep(200);
        while (colorSensor2.blue() < 8&&colorSensor2.red()<8) {
            driveForever(0.2);
            waitOneFullHardwareCycle();
        }
        resetEncoderDelta();
        turn(0); //for some reason it doesn't execute anything until a turn is called :/ so this is necessary
        sleep(200);
        drive(200, 0.5, false);
        sleep(200);
        squareUp();
        sleep(200);

        turn(-75);
        //pivotleft(800);


        sleep(200);

        while (colorSensor2.alpha() < 30) {
            driveForever(0.2);
            waitOneFullHardwareCycle();
        }

        stopMotors();
        sleep(200);
        turn(0, true);
        sleep(200);
        drive(50,0.5);
        stopMotors();
        climberDumper.setPosition(1);
        sleep(1000);
        climberDumper.setPosition(0);
        sleep(200);
        drive(300,-0.2);

        while (colorSensor2.blue() < 8&&colorSensor2.red()<8) {
            driveForever(-0.2);
            waitOneFullHardwareCycle();
        }

        turn(45, true);
        drive(2150,-1);
        turn(-90);
        drive(7000,-0.2, false, false);


     //  turn(-10 * turnDirection);

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
//        drive(-200,0.2);
//        squareUp();
//        sleep(200);
//
//        drive(100, -0.5);
//        sleep(200);
//
//        turn(-80);
//        collector.setPower(1);
//        sleep(200);
//        driveUntilUltra(15,0.2);
//        stopMotors();
    }


    /**
     * sets all motors to a specific speed
     * @param speed target speed
     */
    void driveForever(double speed) {
        // Start the drive wheel motors at full power
        run_using_encoders();
        setLeftPower(speed);
        setRightPower(speed);
    }

    void pivotleft (double distance) throws InterruptedException {
        run_using_encoders();
        while( !hasLeftReached(distance)) {
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
     * @param leftd target distance
     * @return if the encoders are greater than or equal to the target distance
     */
    //Has the left side reached a certain encoder value.
    boolean hasLeftReached(double leftd) {

        return (Math.abs(FLposition()) > leftd) && (Math.abs(BLposition()) > leftd);
    }

    /**
     * determines if the left side has reached the target disance
     * @param rightd target distance
     * @return if the encoders are greater than or equal to the target distance
     */
    //Has the right side reached a certain encoder value.
    boolean hasRightReached(double rightd) {

        return (Math.abs(FRposition()) > rightd) && (Math.abs(BRposition()) > rightd);
    }

    /**
     * sets the left motors to a specific power, keeping in mind the acceptable limits
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
     * @param power target power
     */
    //Set the motors power.
    void setRightPower(double power) {

        power = clip(power, -1, 1);
        run_using_encoders();
        FR.setPower(power);
        BR.setPower(power);
    }

    /**
     * takes a variable and returns the variable
     * if the variable is outside the one of the bounds, instead it returns that bound
     * @param variable the variable to be clipped
     * @param min the minimum exeptable value
     * @param max the maximum exeptable value
     * @return returns the variable, exept it is inside the exeptable vlues
     */
    //Clip the values.
    double clip(double variable, double min, double max) {

        if (variable < min) variable = min;
        if (variable > max) variable = max;

        return variable;
    }

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
        climberDumper = hardwareMap.servo.get("climberDumper");
        debDumper = hardwareMap.servo.get("dumper");
        colorSensor = hardwareMap.colorSensor.get("cs1");
        colorSensor2 = hardwareMap.colorSensor.get("cs2");
        collector = hardwareMap.dcMotor.get("collect");
        ultra1 = hardwareMap.ultrasonicSensor.get("ultraL");
        ultra2 = hardwareMap.ultrasonicSensor.get("ultraR");
        odm = hardwareMap.opticalDistanceSensor.get("odm");

        //Motors
        FR = hardwareMap.dcMotor.get("fr");
        FL = hardwareMap.dcMotor.get("fl");
        BR = hardwareMap.dcMotor.get("br");
        BL = hardwareMap.dcMotor.get("bl");
        someServo1 = hardwareMap.servo.get("armAngle1");
        someServo2 = hardwareMap.servo.get("armAngle2");
    }

    /**
     * resets the gyro delta
     * @see #heading()
     * @throws InterruptedException
     */
    //Resets the gyro based on the old heading.
    void resetGyro() throws InterruptedException {

        while (heading() != 0) {
            lastgyro = gyroSensor.getHeading();
            waitOneFullHardwareCycle();
        }
    }

    /**
     * uses the gyro to turn
     * @param degrees target degrees
     * @param untilAbs if true it uses the absolute heading insted of the altered one {@link #heading()}
     * @throws InterruptedException
     */
    void turn(int degrees, boolean untilAbs) throws InterruptedException {

        resetGyro();
        degrees *= turnDirection; //Changes the turn direction based on team color.
        int dir;

        if(untilAbs==false)
            dir = heading(); //Heading.
        else
            dir = gyroSensor.getHeading(); //Heading.

        //Turn while the difference until gyro lines up with angle.
        while(Math.abs(degrees - dir)>1) {
            telemetry.addData("difference", " " + Math.abs(degrees-dir));
            telemetry.addData("absolute heading", " " + gyroSensor.getHeading());
            telemetry.addData("old gyro", lastgyro);
            if(untilAbs==false)
                dir = heading(); //Heading.
            else
                dir = gyroSensor.getHeading(); //Heading.

            if (dir > 180)
                dir -= 360;

            double power=Math.pow(((degrees-dir)/50),2);
            if(degrees<0)
                power*=-1;
            telemetry.addData("initial power", " " + power);
            power=clip(power,-0.50,0.50);
            if(Math.abs(power)<0.1){
                if((degrees-dir)>0)
                    power= 0.1;
                else
                    power= -0.1;
            }

            telemetry.addData("power", " " + power);

            setLeftPower(-power);
            setRightPower(power);
            waitOneFullHardwareCycle();
        }
        stopMotors();
    }

    /**
     * allows {@link #turn(int, boolean)} to be run without referencing absolute heading
     * @param degrees target degrees
     * @throws InterruptedException
     */
    void turn(int degrees) throws InterruptedException {
        turn(degrees, false);
    }

    /**
     * gets an adjusted headier that can be reset
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
     * @param sensor the sensor to be read
     * @return the more accurate ultrasonic value
     * @throws InterruptedException
     */
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

    /**
     * uses two ultrasonic sensors to square up against a wall
     * @see #readFixedUltra(UltrasonicSensor)
     * @throws InterruptedException
     */
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

    /**
     * stops all motors and verifies that they are stopped
     * @throws InterruptedException
     */
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

    /**
     * drives until the ultrasonic sensors read a certain value
     * @see #readFixedUltra(UltrasonicSensor)
     * @param target target distance
     * @param speed movement speed
     * @throws InterruptedException
     */
    void driveUntilUltra(int target, double speed) throws InterruptedException {
        while (readFixedUltra(ultra1) > target || readFixedUltra(ultra1) < 1) {
            driveForever(speed);
            waitOneFullHardwareCycle();
        }
        stopMotors();
    }

    /**
     * uses encoders, ultrasonic sensors and gyro sensors to drive accurately
     * encoders are used for distance
     * ultrasonic sensors do collision avoidance {@link #blocked()}
     * @param distance distance to travel in encoder clicks
     * @param speed speed to travel at
     * @param avoidance if true, it will use collision avoidance
     * @param correction if true, it will do gyro aided corce correction
     * @throws InterruptedException
     */
    void drive(float distance, double speed, boolean avoidance, boolean correction) throws InterruptedException {
        resetEncoderDelta();
        resetGyro();
        // Start the drive wheel motors at full power
        while (!hasLeftReached(distance) && !hasRightReached(distance)) {
            double turnheading;
            double currSpeed = speed;
            // telemetry.addData("encoder values", "right:" + FR.getCurrentPosition() + " left:" + FL.getCurrentPosition());
            if (correction==true) {
                turnheading = heading();
                if (turnheading > 180)
                    turnheading -= 360;
                turnheading /= 15;

                if (Math.abs(turnheading) > 1)
                    currSpeed = clip(currSpeed, -0.7, 0.7);

                if (blocked() && speed > 0 && avoidance)
                    currSpeed = 0;

                else if (turnheading != 0)
                    currSpeed = clip(currSpeed, -0.9, 0.9);

                telemetry.addData("heading ", "" + heading());
                telemetry.addData("absolute heading", " " + gyroSensor.getHeading());
            }
            else
                turnheading=0;
            run_using_encoders();
            setLeftPower(currSpeed + turnheading);
            setRightPower(currSpeed - turnheading);
            waitOneFullHardwareCycle();
        }
        stopMotors();
        reset_drive_encoders();
    }

    /**
     * runs {@link #drive(float, double, boolean, boolean)} without needing to input
     * collision avoidance or course correction
     * @param distance distance to travel in encoder clicks
     * @param speed speed to travel at
     * @throws InterruptedException
     */
    //Compressed the two drives into one for simplicity - "Ethan ;)"
    void drive(float distance, double speed) throws InterruptedException {
        drive(distance, speed, true, true);
    }

    /**
     * runs {@link #drive(float, double, boolean, boolean)} without needing to input
     * course correction
     * @param distance distance to travel in encoder clicks
     * @param speed speed to travel at
     * @throws InterruptedException
     */
    void drive(float distance, double speed, boolean avoidance) throws InterruptedException {
        drive(distance, speed, avoidance, true);
    }

    /**
     * determines whether the robot is blocked
     * @see #readFixedUltra(UltrasonicSensor)
     * @return if something is detected within 20 cm, TRUE
     * @throws InterruptedException
     */
    boolean blocked() throws InterruptedException {
        return (readFixedUltra(ultra1) < 20 || readFixedUltra(ultra2) < 20);
    }

    /**
     * gets adjusted position of the front right motor
     * reset to 0 using {@link #resetEncoderDelta()}
     * @return the adjusted value
     */
    int FRposition() {
        return (FR.getCurrentPosition() - FRold);
    }

    /**
     * gets adjusted position of the back right motor
     * reset to 0 using {@link #resetEncoderDelta()}
     * @return the adjusted value
     */
    int BRposition() {
        return (BR.getCurrentPosition() - BRold);
    }

    /**
     * gets adjusted position of the front left motor
     * reset to 0 using {@link #resetEncoderDelta()}
     * @return the adjusted value
     */
    int FLposition() {
        return (FL.getCurrentPosition() - FLold);
    }

    /**
     * gets adjusted position of the back left motor
     * reset to 0 using {@link #resetEncoderDelta()}
     * @return the adjusted value
     */
    int BLposition() {
        return (BL.getCurrentPosition() - BLold);
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