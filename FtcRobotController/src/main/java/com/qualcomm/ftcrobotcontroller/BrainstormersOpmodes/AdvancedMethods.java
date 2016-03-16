package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

/**
 * Created by August on 3/5/2016.
 */
public abstract class AdvancedMethods extends AutonomousBuildingBlocks {
    void drive(int distance, double power, int targetType) throws InterruptedException {
        resetEncoderDelta();
        final double oldGyro =  adaFruitGyro.getYaw();
        final double DEVIATIONGAIN = 0.23; //how much deviation effects the robot
        double deviation = 0;
        boolean hasReached = false;
        while(!hasReached){
            double yaw = (adaFruitGyro.getYaw() - oldGyro) % 360; //calculates the angle based on where the robot is now and how far it has to go

            if (yaw > 180) { //determines which way the robot will turn (left or right)
                yaw = yaw - 360.0;
            } else if (yaw < -180) {
                yaw = 360.0 + yaw;
            }
            telemetry.addData("encoder positions", " back right "+ brPosition()+" back left "+ blPosition());
            telemetry.addData("yaw", yaw);
            telemetry.addData("deviation", deviation);
            deviation =  ( yaw * DEVIATIONGAIN );
            if (deviation != 0) {
                deviation = Math.sqrt(Math.abs(deviation)) * (deviation / Math.abs(deviation));
            }

            runUsingEncoders();
            double leftPower = -deviation * Math.abs(power*power);
            double rightPower = deviation * Math.abs(power*power);
            leftPower += power;
            rightPower += power;
            setLeftPower(clip(leftPower, -1, 1));
            setRightPower(clip(rightPower, -1, 1));
            waitOneFullHardwareCycle();

            switch (targetType) {  //determines whether or not to stop when the color sensor detects white
                case 0:
                    hasReached = Math.abs(brPosition() + blPosition()) / 2 > Math.abs(distance);
                    break;
                case 1:
                    hasReached = colorSensor.alpha() >= 2 || Math.abs(brPosition() + blPosition()) / 2 > Math.abs(distance);
                    telemetry.addData("alpha", colorSensor.alpha());
                    break;
                default:
                    telemetry.addData("Invalid input", "stopping");
                    hasReached = true;
                    break;
            }


        }
        stopMotors();
    }

    void pivot (double degrees, int wheels, double tolerance) throws InterruptedException{ //if wheels = 1, use left wheels for blue, if wheels = -1 use right wheels for blue

        final double GAIN = 0.03;

        int countWithinTolerence = 0;
        int count = 0;
        double heading;
        double difference;
        double cyclesMaxPower = 0;
        double power;
        boolean rightTurn = false;
        boolean hasTurned = false;
        degrees *= turnDirection;

        runUsingEncoders();//0s gyro

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
            power = clip(power, 0.1, 0.5);

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

               // stopMotors();
            } else if (difference > 0) {
                countWithinTolerence = 0;

                if(wheels *turnDirection == 1){
                    setLeftPower(power);
                }
                else {
                    setRightPower(-power);
                }
            } else {
                countWithinTolerence = 0;
                if(wheels *turnDirection == 1){
                    setLeftPower(-power);
                }
                else {
                    setRightPower(power);
                }
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

        runUsingEncoders();//0s gyro

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
            power = clip(power, 0.05, 0.35);

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

    void driveUntilUltra(int target, double speed, int maxdistance) throws InterruptedException { //drives until the robot gets within a certain distance of an object
        while (readFixedUltra(ultra2) > target || readFixedUltra(ultra2) < 1) {
            driveForever(speed);
            waitOneFullHardwareCycle();
        }
        setLeftPower(0);
        setRightPower(0);
    }

    void driveForever(double speed) {
        // Start the drive wheel motors at full power
        runUsingEncoders();
        setLeftPower(speed);
        setRightPower(speed);
    }


}
