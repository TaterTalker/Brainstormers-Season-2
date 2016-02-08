package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by August on 2/8/2016.
 */
public abstract class AutonomousMethods extends AutonomousBuildingBlocks {
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
        run_using_encoders();
        // Start the drive wheel motors at full power
        setLeftPower(0.5); //otherwise when calculating turn heading from encoders all will be null and program will halt
        setRightPower(0.5);
        sleep(20);
        do {
            telemetry.addData("drive", "working");
            double turnheading = 0;
            double currSpeed = speed;
            // telemetry.addData("encoder values", "right:" + FR.getCurrentPosition() + " left:" + FL.getCurrentPosition());
            if (correction == true) {
                //turnheading = heading();
                turnheading += (
                        FLposition() +
                                BLposition() -
                                FRposition() -
                                BRposition()
                ) / 5;

                if (turnheading > 180) {
                    turnheading -= 360;
                }
                turnheading /= 15;

                if (Math.abs(turnheading) > 0.5) {
                    currSpeed = clip(currSpeed, -0.7, 0.7);
                }

                if (blocked() && speed > 0 && avoidance) {
                    currSpeed = 0;
                }

                if (turnheading != 0) {
                    currSpeed = clip(currSpeed, -0.9, 0.9);
                }

                telemetry.addData("heading ", "" + heading());
                telemetry.addData("absolute heading", " " + gyroSensor.getHeading());
            } else {
                turnheading = 0;
            }
            telemetry.addData("encoder values", " FL " + FLposition() + " BL " + BLposition() + " FR " + FRposition() + " BR " + BRposition());
            run_using_encoders();
            setLeftPower(currSpeed + turnheading);
            setRightPower(currSpeed - turnheading);
            waitOneFullHardwareCycle();
        } while (!hasLeftReached(distance) && !hasRightReached(distance));
        telemetry.addData("drive", "complete");
        stopMotors();
        sleep(50);
    }

    void driveUnilLight(float val, double speed, boolean avoidance, boolean correction) throws InterruptedException {
        resetEncoderDelta();
        resetGyro();
        run_using_encoders();
        // Start the drive wheel motors at full power
        setLeftPower(0.5); //otherwise when calculating turn heading from encoders all will be null and program will halt
        setRightPower(0.5);
        sleep(20);
        do {
            telemetry.addData("drive", "working");
            double turnheading = 0;
            double currSpeed = speed;
            // telemetry.addData("encoder values", "right:" + FR.getCurrentPosition() + " left:" + FL.getCurrentPosition());
            if (correction == true) {
                //turnheading = heading();
                turnheading += (
                        FLposition() +
                                BLposition() -
                                FRposition() -
                                BRposition()
                ) / 5;

                if (turnheading > 180) {
                    turnheading -= 360;
                }
                turnheading /= 15;

                if (Math.abs(turnheading) > 0.5) {
                    currSpeed = clip(currSpeed, -0.7, 0.7);
                }

                if (blocked() && speed > 0 && avoidance) {
                    currSpeed = 0;
                }

                if (turnheading != 0) {
                    currSpeed = clip(currSpeed, -0.9, 0.9);
                }

                telemetry.addData("heading ", "" + heading());
                telemetry.addData("absolute heading", " " + gyroSensor.getHeading());
            } else {
                turnheading = 0;
            }
            telemetry.addData("encoder values", " FL " + FLposition() + " BL " + BLposition() + " FR " + FRposition() + " BR " + BRposition());
            run_using_encoders();
            setLeftPower(currSpeed + turnheading);
            setRightPower(currSpeed - turnheading);
            waitOneFullHardwareCycle();
        } while (colorSensor2.alpha() < val);
        stopMotors();
        sleep(50);
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
}
