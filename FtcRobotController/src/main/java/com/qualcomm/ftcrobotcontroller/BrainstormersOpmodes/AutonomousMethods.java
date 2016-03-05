package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by August & Ethan & Dan on 12/1/2015.
 */

public abstract class AutonomousMethods extends AutonomousBuildingBlocks {
    /**
     * uses the gyro to turn
     *
     * @param degrees target degrees
     * @throws InterruptedException
     */
    void turnTo(int degrees, int tolerance) throws InterruptedException {
        int heading, difference, countwithintolerence = 0, count = 0, cyclesMaxPower = 0;
        degrees *= turnDirection;
        double power;
        boolean rightTurn = false;
        run_using_encoders();

        while (true) { //while the turn hasn't been completed we run through this loop
            count++;
            heading = gyroSensor.getHeading();
            difference = (degrees - heading) % 360; //calculates the angle based on where the robot is now and how far it has to go

            if (difference > 180) { //determines which way the robot will turn (left or right)
                difference = difference - 360;
            } else if (difference < -180) {
                difference = 360 + difference;
            }

            telemetry.addData("Heading", " " + heading + " " + difference + " " + rightTurn); //determines how fast the turn should be, as the turn gets greater the speed gets faster
            power = Math.abs(difference / 45);
            power = clip(power, 0.075, 0.35);

            if (cyclesMaxPower == 0) {
                cyclesMaxPower = Math.abs(difference / 30) + 3;
            }

            if (count < cyclesMaxPower && Math.abs(difference) > 20) { //speeds up out power to speed up turn during the beginning of the turn
                power = 1;
            }

            if (Math.abs(difference) <= tolerance) { //how far off the turn can be while still being successful (tolerance of turn)
                countwithintolerence++;
                if (countwithintolerence > 15) {
                    break;
                }
                setLeftPower(0);
                setRightPower(0);
            } else if (difference > 0) {
                countwithintolerence = 0;
                setLeftPower(power);
                setRightPower(-power);
                rightTurn = true;
            } else {
                countwithintolerence = 0;
                setLeftPower(-power);
                setRightPower(power);
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

    /**
     * drives until the ultrasonic sensors read a certain value
     *
     * @param target target distance
     * @param speed  movement speed
     * @throws InterruptedException
     * @see #readFixedUltra(UltrasonicSensor)
     */
    void driveUntilUltra(int target, double speed, int maxdistance) throws InterruptedException { //drives until the robot gets within a certain distance of an object
        while (readFixedUltra(ultra2) > target || readFixedUltra(ultra2) < 1) {
            driveForever(speed);
            waitOneFullHardwareCycle();
        }
        stopMotors();
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
    void drive(float distance, double speed, boolean avoidance, boolean correction, int targetType) throws InterruptedException {
        int count = 0; //how many times the loop has run
        int blockedcount = 0; //determines how long the robot has been blocked for
        resetEncoderDelta();
        resetGyro();

        //start the drive wheel motors at full power otherwise when calculating turn heading from encoders all will be null and program will halt
        run_using_encoders();
        setLeftPower(0.5);
        setRightPower(0.5);

        sleep(20);
        boolean isComplete = false;
        do {
            count++;
            telemetry.addData("drive", "working");
            double turnheading = 0;
            double currSpeed = speed;

            if (correction == true) { //using encoder values to correct offsets/drift
                turnheading += (
                        FLposition() +
                                BLposition() -
                                FRposition() -
                                BRposition()
                ) / 80.0; //MUST BE FLOAT

                //ramping up speed to avoid jerk
                if (Math.abs(turnheading) > 0.5 || count < 50) {
                    currSpeed = clip(currSpeed, -0.6, 0.6);
                }

                if (blocked() && speed > 0 && avoidance) { //if the robot is blocked, it will stop
                    blockedcount++;
                    if (blockedcount > 3) {
                        currSpeed = 0;
                        telemetry.addData("Blocked", "Blocked!!!!");
                    }
                } else {
                    blockedcount = 0; //resets block count after the robot is no longer blocked
                }

                if (turnheading != 0) {
                    currSpeed = clip(currSpeed, -0.95, 0.95);
                }

                telemetry.addData("heading ", "" + heading());
                telemetry.addData("absolute heading", " " + gyroSensor.getHeading());
                telemetry.addData("deviation", turnheading);
            } else {
                turnheading = 0;
            }


            turnheading = clip(turnheading, -0.25, 0.25); //clips the turn heading

            //sets the power (repeated to avoid glitching)
            telemetry.addData("encoder values", " FL " + FLposition() + " BL " + BLposition() + " FR " + FRposition() + " BR " + BRposition());
            run_using_encoders();
            setLeftPower(currSpeed + turnheading * currSpeed);
            setRightPower(currSpeed - turnheading * currSpeed);
            telemetry.addData("left power ", +(currSpeed + turnheading * currSpeed) + " right power: " + (currSpeed - turnheading * currSpeed));
            telemetry.addData("encoder values", " FL " + FLposition() + " " + FL.getCurrentPosition() + " BL " + BLposition() + " " + BL.getCurrentPosition() + " FR " + FRposition() + " " + FR.getCurrentPosition() + " BR " + BRposition() + " " + BR.getCurrentPosition());
            run_using_encoders();
            turnheading = clip(turnheading, -1, 1);
            setLeftPower(currSpeed + turnheading * (currSpeed));
            setRightPower(currSpeed - turnheading * (currSpeed));
            telemetry.addData("left power ", +(currSpeed + turnheading * currSpeed) + " right power: " + (currSpeed - turnheading * currSpeed));

            waitOneFullHardwareCycle();

            switch (targetType) {  //determines whether or not to stop when the color sensor detects white
                case 0:
                    isComplete = (hasLeftReached(distance) || hasRightReached(distance));
                    break;
                case 1:

                    boolean colorvalue = false;
                    try{
                      colorvalue = colorSensor2.alpha() > 1;

                    }catch(Exception fuck){

                    }

                    isComplete = colorvalue || hasLeftReached(distance) || hasRightReached(distance);
                    telemetry.addData("alpha", colorSensor2.alpha());
                    break;
                default:
                    telemetry.addData("Invalid input", "stopping");
                    isComplete = true;
                    break;
            }

        }

        while (isComplete == false); //ends the drive function
        telemetry.addData("drive", "complete");
        stopMotors();
        sleep(50);
    }
}
