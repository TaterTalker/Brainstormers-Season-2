package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;
/**
 * contains all the code to run Autonomous, it has no inherent side
 */
public abstract class AutonomousLinearBotmk2 extends AdvancedMethods {
    /**
     * run all other functions to perform autonomous
     * @param turnDirectionInput 1=blue -1=red
     * @throws InterruptedException
     */
    boolean startNearRamp=false;  //Decides where starting position is

    public void runOpMode(int turnDirectionInput) throws InterruptedException {
        telemetry.addData("Init", "running");

        turnDirection = turnDirectionInput; //adjusts turns based on team color
        getRobotConfig();//Map Motors and Sensors

        //Configure and Reset
        runUsingEncoders();
        resetDriveEncoders();
        //  gyroSensor.calibrate();
        climberDumper.setPosition(0.5);
        armAngle1.setPosition(0.5);
        armAngle2.setPosition(0.5);
        sideArmL.setPosition(0.75);
        sideArmR.setPosition(0);
        doorR.setPosition(0.85);
        doorL.setPosition(0.15);
        beaconR.setPosition(0);
        beaconL.setPosition(0);
        debDumper.setPosition((turnDirection + 1) / 2);
        sleep(5000);
        telemetry.addData("Init", "done");
        boolean triggerBeacon=true;
        cameraController.startCam();

        while (!gamepad1.a && !gamepad1.b) {//adds in delay from button press
            if(gamepad1.y) {
                triggerBeacon = false;
            }
            if (gamepad1.dpad_up) {
                delay = delay + 1000;
            }
            else if (gamepad1.dpad_down) {
                delay = delay - 1000;
            }
            telemetry.addData("Delay Seconds:", delay / 1000);
            sleep(250);
        }

        if (gamepad1.a)  //sets starting position of robot
        {
            startNearRamp = true;
        }

        if (startNearRamp) {
            telemetry.addData("Near Ramp", "");
        }
        else {
            telemetry.addData("Far From Ramp", "");
        }
        telemetry.addData("Ready", "");
        if(triggerBeacon){
            telemetry.addData("Beacon", "Activated");
        }
        else{
            telemetry.addData(" No Beacon", "Deactivated");
        }


        while (!isStarted()) {
            adaFruitGyro.initIMU();
        }

        waitForStart(); //everything before this happens when you press init

        beacon.setPosition(1);
        sleep(300);
        sleep(delay);
        collector.setPower(-0.7);
        climberDumper.setPosition(0.5);
        if (startNearRamp) { //near ramp position
            piDrive(1600, .7, 0);
            if (turnDirectionInput == 1) {
                pivot(36,1, 1);
            } else {
                pivot(37,-1, 1);
            }
            piDrive(4600, 1, 0);
        }
        else { //far ramp position
            piDrive(1600, .7, 0);
            if (turnDirectionInput == 1) {
                newGyroTurn(50, 1);
            } else {
                newGyroTurn(51, 1);
            }
            piDrive(7000, 1, 0);
        }
        newGyroTurn(45, 2);
        piDrive(700, .20,1);
        piDrive(200,.2,0);
        pivot(88,1, 1);


        stopMotors();
        collector.setPower(0);
        driveUntilUltra(30, 0.1, 1200);
        //use camera to analyze the image and get the left and right red values
        if (triggerBeacon) {
            sleep(100);
            int leftred = cameraController.getLeftRed();
            int rightred = cameraController.getRightRed();

            telemetry.addData("Colors", "Left " + leftred / 1000 + " Right: " + rightred / 1000);
            if (leftred > rightred) //left side is red
                if (turnDirection == -1)
                    beaconR.setPosition(0.3);
                else
                    beaconR.setPosition(0.7);
            else //right side is red
                if (turnDirection == -1)
                    beaconR.setPosition(0.7);
                else
                    beaconR.setPosition(0.3);
            sleep(500);
        }
        driveUntilUltra(15, 0.1, 200);
        waitForNextHardwareCycle();
        driveUntilUltra(15, 0.1, 200);
        climberDumper.setPosition(0.75);
        sleep(800);
        climberDumper.setPosition(0);
        sleep(200);
        climberDumper.setPosition(0.5);
        piDrive(500, -0.25,0);
        beaconR.setPosition(0.9);
        pivot(-180,-1, 2);
        collector.setPower(1);
        piDrive(3000, 1,0);
        collector.setPower(1);
        newGyroTurn(135, 2);
        piDrive(1000,1,0);
        /*
        if (turnDirectionInput == 1) {
            turnTo(-52, 1);
        }
        else{
            turnTo(-55, 1);
        }

        //sets the beacon trigger arm depending on team color.
        if (turnDirectionInput ==1){
            sideArmR.setPosition(0.6);
        }
        else {
            sideArmL.setPosition(0.1);
        }

        //drive(10000, -0.7, false, false, 0); //climbs ramp.
        telemetry.addData("Red, Blue", " " + colorSensor.blue() + " " + colorSensor.red());
        sleep(100);
        waitOneFullHardwareCycle();
        */
    }

    //separate turn test function not in use.
    public void turnTest() throws InterruptedException {
        long start= System.currentTimeMillis();
        newGyroTurn(36, 0.5);
        newGyroTurn(25, 0.5);
        newGyroTurn(88, 0.5);
        newGyroTurn(-170, 0.5);
        newGyroTurn(135, 0.5);
        newGyroTurn(-52, 0.5);
        long end=System.currentTimeMillis();
        telemetry.addData("timing", Long.toString(end-start));
        sleep(150000);
    }
}
