package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

/**
 * Created by August on 3/5/2016.
 */
public abstract class AdvancedMethods extends AutonomousBuildingBlocks {
    void piDrive(int distance, double power) throws InterruptedException {
        adaFruitGyro.startIUM();
        resetEncoderDelta();
        final double DEVIATIONGAIN = 0.3; //how much deviation effects the robot
        double overShoot = 0;
        double deviation = 0;
        boolean hasReached = false;
        while(hasReached == false){
            double yaw = adaFruitGyro.getYaw();
            telemetry.addData("encoder positions", " back right "+ brPosition()+" back left "+ blPosition());
            telemetry.addData("yaw", yaw);
            telemetry.addData("deviation", deviation);
            deviation =  ( yaw * DEVIATIONGAIN );
            if (deviation != 0) {
                deviation = Math.sqrt(Math.abs(deviation)) * (deviation / Math.abs(deviation));
            }

            runUsingEncoders();
            double leftPower = -deviation;
            double rightPower = deviation;
            leftPower += power;
            rightPower += power;
            setLeftPower(clip(leftPower, -1, 1));
            setRightPower(clip(rightPower, -1, 1));
            hasReached = Math.abs(brPosition() + blPosition()) / 2 > Math.abs(distance);
            waitOneFullHardwareCycle();
        }
        stopMotors();
    }

    void newGyroTurn(double degrees, double tolerance) throws InterruptedException {

        final double GAIN = 0.015;

        int countWithinTolerence = 0;
        int count = 0;
        double heading;
        double difference;
        double cyclesMaxPower = 0;
        double power;
        boolean rightTurn = false;
        boolean hasTurned = false;
        degrees *= turnDirection;

        runUsingEncoders();
        adaFruitGyro.startIUM(); //0s gyro

        while (!hasTurned) { //while the turn hasn't been completed we run through this loop
            count++;
            heading = adaFruitGyro.getYaw();
            difference = (degrees - heading) % 360; //calculates the angle based on where the robot is now and how far it has to go

            if (difference > 180) { //determines which way the robot will turn (left or right)
                difference = difference - 360.0;
            } else if (difference < -180) {
                difference = 360.0 + difference;
            }

            telemetry.addData("Heading", " " + heading + " " + difference + " " + rightTurn); //determines how fast the turn should be, as the turn gets greater the speed gets faster
            power = Math.abs(difference*GAIN);
            power = clip(power, 0.03, 0.35);

            if (cyclesMaxPower == 0) {
                cyclesMaxPower = Math.abs(difference / 30.0) + 3;
            }

            if (count < cyclesMaxPower && Math.abs(difference) > 20) { //speeds up out power to speed up turn during the beginning of the turn
                power = 1;
                telemetry.addData("1", "power");
            }

            telemetry.addData("power", "" + power);
            if (Math.abs(difference) <= tolerance) { //how far off the turn can be while still being successful (tolerance of turn)
                countWithinTolerence++;
                if (countWithinTolerence > 15) {
                    hasTurned = true;
                }
                setLeftPower(0);
                setRightPower(0);
            } else if (difference > 0) {
                countWithinTolerence = 0;
                setLeftPower(power);
                setRightPower(-power);
                rightTurn = true;
            } else {
                countWithinTolerence = 0;
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


}
