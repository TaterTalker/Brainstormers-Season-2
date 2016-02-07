package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

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

/**
 * contains all the code to run Autonomous, it has no inherent side
 */
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

    //Our Main Function that Controls the Robots Actions

    /**
     * run all other functions to perform autonomous
     *
     * @param turnDirectionInput 1=blue -1=red
     * @throws InterruptedException
     */
    public void runOpMode(int turnDirectionInput) throws InterruptedException {
        telemetry.addData("Init", "running");
        //Adjusts turns based on team color.
        turnDirection = turnDirectionInput;

        //Map Motors and Sensors.
        getRobotConfig();

        //Configure and Reset.
        run_using_encoders();
        reset_drive_encoders();
        gyroSensor.calibrate();
        climberDumperB.setPosition(0);
      //  climberDumperR.setPosition(1);
        armAngle1.setPosition(0.5);
        armAngle2.setPosition(0.5);
       // sideArmL.setPosition(0.05);
      //  sideArmR.setPosition(.75);
        doorL.setPosition(0.3);
        doorR.setPosition(0.8);
        debDumper.setPosition((turnDirection + 1) / 2);


        sleep(5000);
        telemetry.addData("Init", "done");
        waitForStart(); //everything before this happens when you press init


        //collector.setPower(-1);
        final boolean SLEEP = false;

        //pivotleft(200);
//        if (SLEEP) sleep(5000);
//        turnTo(-86);
//        sleep(1000);
//        turnTo(-172);
//        sleep(1000);
//        turnTo(-258);
//        sleep(500);
//        turnTo(-344);
//        sleep(100000);
        drive(1000, 1);
        sleep(500);

        if (SLEEP) sleep(5000);
        drive(1000, 1);
        sleep(1000);


        turn(45);
        sleep(200);
        drive(300, 0.2);

        driveUntilUltra(17, 0.2);
        sleep(100);
        squareUp();
        sleep(100);
        turn(-90);

        resetGyro();
        while (colorSensor2.alpha() < 15) {
            driveForeverWitGyro(0.2);
            waitOneFullHardwareCycle();
        }
        stopMotors();
        reset_drive_encoders();
        resetEncoderDelta();
        stopMotors();
        sleep(100);
        drive(20, -0.5);
        drive(1000, -0.5);
        stopMotors();
        //turn(10);
        if (turnDirection == -1) {
            climberDumperB.setPosition(1);


        }
       else {

            climberDumperB.setPosition(0);
            //climberDumperR.setPosition(0);
        }
        sleep(1000);
        climberDumperB.setPosition(0);
        //climberDumperR.setPosition(1);
        sleep(200);
        while (colorSensor2.alpha() < 10) {
            driveForever(0.2);
            waitOneFullHardwareCycle();
        }

        reset_drive_encoders();
        turn(-135);
        collector.setPower(0.8);

        drive(3150, 1, false, false);

        //STUFF FROM HERE DOWN IS IN FRONT TO THE RAMP
        turn(-69);
        drive(1000, 1);
        drive(1000, -1);
        sleep(1000);
        turn(90);
        turn(90);


        //  turn(-10 * turnDirection);

        telemetry.addData("Red, Blue", " " + colorSensor2.blue() + " " + colorSensor2.red());
        sleep(100);
        waitOneFullHardwareCycle();
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
     * uses the gyro to turn
     *
     * @param degrees target degrees
     * @throws InterruptedException
     */
    void turnTo(int degrees) throws InterruptedException {
        int heading, difference, count = 0;
        double power;
        boolean rightTurn = false;
        run_using_encoders();
        while (true) {
            heading = gyroSensor.getHeading();
            difference = (degrees - heading) % 360;
            if (difference > 180)
                difference = difference - 360;
            else if (difference < -180)
                difference = 360 + difference;
            telemetry.addData("Heading", " " + heading + " " + difference + " " + rightTurn);
            power = Math.abs(difference / 100.0);
            power = clip(power, 0.05, 0.2);
            if (difference == 0) {
                count++;
                if (count > 100)
                    break;
                setLeftPower(0);
                setRightPower(0);
            } else if (difference > 0) {
                setLeftPower(-power);
                setRightPower(power);
                rightTurn = true;
            } else {
                setLeftPower(power);
                setRightPower(-power);
                rightTurn = false;
            }
            waitOneFullHardwareCycle();
        }
        telemetry.addData("Do", "ne");
        setLeftPower(0);
        setRightPower(0);
        sleep(20);
        setLeftPower(0);
        setRightPower(0);

    }

    void turn(int degrees, boolean untilAbs) throws InterruptedException {
        resetGyro();
        degrees *= turnDirection;
        int headingDelta = gyroSensor.getHeading() - lastgyro;
        double defaultpower = 0.1;

        int distToTarget = degrees - headingDelta;
        do {
            telemetry.addData("heading delta", headingDelta);
            telemetry.addData("distance to target", distToTarget);
            telemetry.addData("absolute heading", gyroSensor.getHeading());
            telemetry.addData("old gyro", lastgyro);
            headingDelta = gyroSensor.getHeading() - lastgyro;

            while (headingDelta > 180) {
                headingDelta -= 360;
            }

            while (headingDelta < -180) {
                headingDelta += 360;
            }

            distToTarget = degrees - headingDelta;


            if (distToTarget > 0) {
                setLeftPower(-defaultpower - distToTarget / 100);
                setRightPower(defaultpower + distToTarget / 100);
            }
            if (distToTarget < 0) {
                setLeftPower(defaultpower + distToTarget / 100);
                setRightPower(-defaultpower - distToTarget / 100);
            }


            waitOneFullHardwareCycle();
        } while (distToTarget != 0);
        stopMotors();
        resetGyro();
        reset_drive_encoders();
        sleep(100);


    }

    /**
     * allows {@link #turn(int, boolean)} to be run without referencing absolute heading
     *
     * @param degrees target degrees
     * @throws InterruptedException
     */
    void turn(int degrees) throws InterruptedException {
        turn(degrees, false);
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
     *
     * @param target target distance
     * @param speed  movement speed
     * @throws InterruptedException
     * @see #readFixedUltra(UltrasonicSensor)
     */
    void driveUntilUltra(int target, double speed) throws InterruptedException {
        while (readFixedUltra(ultra1) > target || readFixedUltra(ultra1) < 1) {
            driveForever(speed);
            waitOneFullHardwareCycle();
        }
        stopMotors();
    }

    void driveForeverWitGyro(double speed) throws InterruptedException {
        double turnheading;
        double currSpeed = speed;
        // telemetry.addData("encoder values", "right:" + FR.getCurrentPosition() + " left:" + FL.getCurrentPosition());
        turnheading = heading();
        if (turnheading > 180)
            turnheading -= 360;
        turnheading /= 15;

        if (Math.abs(turnheading) > 3)
            currSpeed = clip(currSpeed, -0.9, 0.9);

        else if (turnheading != 0)
            currSpeed = clip(currSpeed, -0.95, 0.95);

        telemetry.addData("heading ", "" + heading());
        telemetry.addData("absolute heading", " " + gyroSensor.getHeading());
        run_using_encoders();
        setLeftPower(currSpeed + turnheading / 3);
        setRightPower(currSpeed - turnheading / 3);
    }

    /**
     * uses encoders, ultrasonic sensors and gyro sensors to drive accurately
     * encoders are used for distance
     * ultrasonic sensors do collision avoidance {@link #blocked()}
     *
     * @param distance   distance to travel in encoder clicks
     * @param speed      speed to travel at
     * @param avoidance  if true, it will use collision avoidance
     * @param correction if true, it will do gyro aided course correction
     * @throws InterruptedException
     */
    void drive(float distance, double speed, boolean avoidance, boolean correction) throws InterruptedException {
        resetEncoderDelta();
        resetGyro();
        // Start the drive wheel motors at full power
        setLeftPower(0.5); //otherwise when calculating turn heading from encoders all will be null and program will halt
        setRightPower(0.5);
        sleep(20);
        do {
            double turnheading;
            double currSpeed = speed;
            // telemetry.addData("encoder values", "right:" + FR.getCurrentPosition() + " left:" + FL.getCurrentPosition());
            if (correction == true) {
                turnheading = heading();
                turnheading += (
                        FL.getCurrentPosition() +
                                BL.getCurrentPosition() -
                                FR.getCurrentPosition() -
                                BR.getCurrentPosition()
                ) / 2;
                sleep(1);//all of these are required to allow for stopping
                if (turnheading > 180) {
                    turnheading -= 360;
                }
                turnheading /= 15;
                sleep(1);

                if (Math.abs(turnheading) > 0.5) {
                    currSpeed = clip(currSpeed, -0.7, 0.7);
                }
                sleep(1);

                if (blocked() && speed > 0 && avoidance) {
                    currSpeed = 0;
                }
                sleep(1);

                if (turnheading != 0) {
                    currSpeed = clip(currSpeed, -0.9, 0.9);
                }
                sleep(1);

                telemetry.addData("heading ", "" + heading());
                telemetry.addData("absolute heading", " " + gyroSensor.getHeading());
            } else {
                turnheading = 0;
            }
            telemetry.addData("encoder values", " FL " + FL.getCurrentPosition() + " BL " + BL.getCurrentPosition() + " FR " + FR.getCurrentPosition() + " BR " + BR.getCurrentPosition());
            run_using_encoders();
            setLeftPower(currSpeed + turnheading);
            setRightPower(currSpeed - turnheading);
            sleep(1);
            waitOneFullHardwareCycle();
        } while (!hasLeftReached(distance) && !hasRightReached(distance));
        stopMotors();
        reset_drive_encoders();
        sleep(100);
    }

    /**
     * runs {@link #drive(float, double, boolean, boolean)} without needing to input
     * collision avoidance or course correction
     *
     * @param distance distance to travel in encoder clicks
     * @param speed    speed to travel at
     * @throws InterruptedException
     */
    //Compressed the two drives into one for simplicity - "Ethan ;)"
    void drive(float distance, double speed) throws InterruptedException {
        drive(distance, speed, true, true);
    }

    /**
     * runs {@link #drive(float, double, boolean, boolean)} without needing to input
     * course correction
     *
     * @param distance distance to travel in encoder clicks
     * @param speed    speed to travel at
     * @throws InterruptedException
     */
    void drive(float distance, double speed, boolean avoidance) throws InterruptedException {
        drive(distance, speed, avoidance, true);
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
        return (FL.getCurrentPosition() - FLold);
    }

    /**
     * gets adjusted position of the back left motor
     * reset to 0 using {@link #resetEncoderDelta()}
     *
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