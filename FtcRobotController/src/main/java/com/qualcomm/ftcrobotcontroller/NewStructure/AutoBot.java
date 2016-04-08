package com.qualcomm.ftcrobotcontroller.NewStructure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by ethan on 4/8/2016.
 */
public class AutoBot extends Robot {

    int side;
    LinearOpMode opMode;

    boolean triggerBeacon;

    boolean startNearRamp=false;  //Decides where starting position is
    boolean goToRamp=true;
    int delay;
    public boolean isStartNearRamp() {
        return startNearRamp;
    }

    public boolean isGoToRamp() {
        return goToRamp;
    }

    public boolean isTriggerBeacon() {
        return triggerBeacon;
    }

    public int getDelay() {
        return delay;
    }

    public AutoBot (int side, LinearOpMode varopMode) throws InterruptedException{
        super(side,varopMode);
        opMode = varopMode;
        opMode.telemetry.addData("Init", "running");


        turnDirection = side; //adjusts turns based on team color
        opMode.telemetry.addData("Init", "running2");
        //Configure and Reset
        wheelBase.runUsingEncoders();
        wheelBase.resetDriveEncoders();
        opMode.telemetry.addData("Init", "running3");
        climberDumper.setPosition(0.5);
        sideArms.initSideArms();
        armHook.setPosition(0);

        dumper.startDumper();
        beaconR.setPosition(0);
        beaconL.setPosition(1);
        arm.setLockDown();
        opMode.telemetry.addData("Init", "running4");
        opMode.telemetry.addData("Init", "done");
        triggerBeacon=true;
        frontCam.startFrontCam();
        opMode.telemetry.addData("Init", "running5");

        while (!opMode.gamepad1.a && !opMode.gamepad1.b) {//adds in delay from button press
            if(opMode.gamepad1.y) {
                triggerBeacon = false;
            }
            if (opMode.gamepad1.dpad_up) {
                delay = delay + 1000;
            }
            else if (opMode.gamepad1.dpad_down) {
                delay = delay - 1000;
            }
            if (opMode.gamepad1.x){ //if pressed gets out of way and avoids ramp
                goToRamp=false;
            }
            opMode.telemetry.addData("trigger beacon ", triggerBeacon);
            opMode.telemetry.addData("go to ramp ", goToRamp);
            opMode.telemetry.addData("Delay Seconds ", delay / 1000);
           opMode.sleep(250);
        }

        if (opMode.gamepad1.a)  //sets starting position of robot
        {
            startNearRamp = true;
        }

        if (startNearRamp) {
            opMode.telemetry.addData("Near Ramp", "");
        }
        else {
            opMode.telemetry.addData("Far From Ramp", "");
        }
        opMode.telemetry.addData("Ready", "");
        if(triggerBeacon){
            opMode.telemetry.addData("Beacon", "Activated");
        }
        else{
            opMode.telemetry.addData(" No Beacon", "Deactivated");
        }
        if (goToRamp){
            opMode.telemetry.addData("going to", "ramp");
        }
        else {
            opMode.telemetry.addData("going to", "place next to ramp");
        }


        while (!adaFruitGyro.initDone) {
            adaFruitGyro.initIMU();
        }


    }

    public void start(){

        sideArms.setSideArmLpos(0.75f);
        dumper.resetDumpingBlock();

        beaconR.setPosition(0.9);
        beaconL.setPosition(0.1);
        climberDumper.setPosition(0.5); //makes sure climber dumper will not move
    }

    void drive(int distance, double power, int targetType) throws InterruptedException {
        wheelBase.resetEncoderDelta();
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
            opMode.telemetry.addData("encoder positions", " back right " + wheelBase.getBrPos() + " back left " + wheelBase.getBlPos());
            opMode.telemetry.addData("yaw", yaw);
            opMode.telemetry.addData("deviation", deviation);
            deviation =  ( yaw * DEVIATIONGAIN );
            if (deviation != 0) {
                deviation = Math.sqrt(Math.abs(deviation)) * (deviation / Math.abs(deviation));
            }

            wheelBase.runUsingEncoders();
            double leftPower = -deviation * Math.abs(power*power);
            double rightPower = deviation * Math.abs(power*power);
            leftPower += power;
            rightPower += power;
            wheelBase.setLeftPower(leftPower);
            wheelBase.setRightPower(rightPower);
            opMode.waitOneFullHardwareCycle();

            switch (targetType) {  //determines whether or not to stop when the color sensor detects white
                case 0:
                    hasReached = Math.abs(wheelBase.getBrPos()+ wheelBase.getBlPos()) / 2 > Math.abs(distance);
                    break;
                case 1:
                    hasReached = colorSensor.alpha() >= 2 || Math.abs(wheelBase.getBrPos()+ wheelBase.getBlPos()) / 2 > Math.abs(distance);
                    opMode.telemetry.addData("alpha", colorSensor.alpha());
                    break;
                default:
                    opMode.telemetry.addData("Invalid input", "stopping");
                    hasReached = true;
                    break;
            }


        }
        wheelBase.stopMotors();
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

        wheelBase.runUsingEncoders();//0s gyro

        while (!hasTurned) { //while the turn hasn't been completed we run through this loop
            count++;
            heading = adaFruitGyro.getYaw();
            difference = (degrees - heading) % 360; //calculates the angle based on where the robot is now and how far it has to go

            if (difference > 180) { //determines which way the robot will turn (left or right)
                difference = difference - 360.0;
            } else if (difference < -180) {
                difference = 360.0 + difference;
            }

            opMode.telemetry.addData("Heading", " " + heading + " " + difference + " " + rightTurn); //determines how fast the turn should be, as the turn gets greater the speed gets faster
            power = Math.abs(difference*GAIN);
            power = Range.clip(power, 0.1, 0.5);

            if (cyclesMaxPower == 0) {
                cyclesMaxPower = Math.abs(difference / 30.0) + 3;
            }

            if (count < cyclesMaxPower && Math.abs(difference) > 20) { //speeds up out power to speed up turn during the beginning of the turn
                power = 1;
                opMode.telemetry.addData("1", "power");
            }

            opMode.telemetry.addData("power", "" + power);
            if (Math.abs(difference) <= tolerance) { //how far off the turn can be while still being successful (tolerance of turn)
                countWithinTolerence++;
                if (countWithinTolerence > 15) {
                    hasTurned = true;
                }
                wheelBase.stopMotors();

                // wheelBase.stopMotors();
            } else if (difference > 0) {
                countWithinTolerence = 0;

                if(wheels *turnDirection == 1){
                    wheelBase.setLeftPower(power);
                }
                else {
                    wheelBase.setRightPower(-power);
                }
            } else {
                countWithinTolerence = 0;
                if(wheels *turnDirection == 1){
                    wheelBase.setLeftPower(-power);
                }
                else {
                    wheelBase.setRightPower(power);
                }
            }

            opMode.waitOneFullHardwareCycle();
        }

        opMode.telemetry.addData("Do", "ne");
        wheelBase.setLeftPower(0);
        wheelBase.setRightPower(0);
        wheelBase.setLeftPower(0);
        wheelBase.setRightPower(0);


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

        wheelBase.runUsingEncoders();//0s gyro

        while (!hasTurned) { //while the turn hasn't been completed we run through this loop
            count++;
            heading = adaFruitGyro.getYaw();
            difference = (degrees - heading) % 360; //calculates the angle based on where the robot is now and how far it has to go

            if (difference > 180) { //determines which way the robot will turn (left or right)
                difference = difference - 360.0;
            } else if (difference < -180) {
                difference = 360.0 + difference;
            }

            opMode.telemetry.addData("Heading", " " + heading + " " + difference + " " + rightTurn); //determines how fast the turn should be, as the turn gets greater the speed gets faster
            power = Math.abs(difference*GAIN);
            power = Range.clip(power, 0.05, 0.35);

            if (cyclesMaxPower == 0) {
                cyclesMaxPower = Math.abs(difference / 30.0);
            }

            if (count < cyclesMaxPower && Math.abs(difference) > 30) { //speeds up out power to speed up turn during the beginning of the turn
                power = 1;
                opMode.telemetry.addData("1", "power");
            }

            opMode.telemetry.addData("power", "" + power);
            if (Math.abs(difference) <= tolerance) { //how far off the turn can be while still being successful (tolerance of turn)
                countWithinTolerence++;
                if (countWithinTolerence > 15) {
                    hasTurned = true;
                }
                wheelBase.setLeftPower(0);
                wheelBase.setRightPower(0);

            } else if (difference > 0) {
                countWithinTolerence = 0;
                wheelBase.setLeftPower(power);
                wheelBase.setRightPower(-power);
                rightTurn = true;
            } else {
                countWithinTolerence = 0;
                wheelBase.setLeftPower(-power);
                wheelBase.setRightPower(power);
                rightTurn = false;
            }

            opMode.waitOneFullHardwareCycle();
        }

        opMode.telemetry.addData("Do", "ne");
        wheelBase.setLeftPower(0);
        wheelBase.setRightPower(0);
        wheelBase.setLeftPower(0);
        wheelBase.setRightPower(0);
    }

    void driveUntilUltra(int target, double speed, int maxdistance) throws InterruptedException { //drives until the robot gets within a certain distance of an object
        while (readFixedUltra(ultra2) > target || readFixedUltra(ultra2) < 1) {
            driveForever(speed);
            opMode.waitOneFullHardwareCycle();
        }
        wheelBase.setLeftPower(0);
        wheelBase.setRightPower(0);
    }

    void driveForever(double speed) {
        // Start the drive wheel motors at full power
        wheelBase.runUsingEncoders();
        wheelBase.setLeftPower(speed);
        wheelBase.setRightPower(speed);
    }

    double readFixedUltra(UltrasonicSensor sensor) throws InterruptedException {

        double val = 0;
        double maxVal = 0;
        double minVal = 255;
        double tmpVal;

        for (int i = 0; i < 4; i++) {
            do {
                tmpVal = sensor.getUltrasonicLevel();
                opMode.waitOneFullHardwareCycle();
            } while (tmpVal == 0);
            if (tmpVal < minVal) {
                minVal = tmpVal;
            }
            if (tmpVal > maxVal) {
                maxVal = tmpVal;
            }
            val += tmpVal;
            opMode.waitOneFullHardwareCycle();
        }
        val -= (minVal + maxVal);
        val /= 2;
        return val;
    }


}
