package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * contains all the code to run Autonomous, it has no inherent side
 */
public abstract class Autonomous extends AdvancedMethods {
    /**
     * run all other functions to perform autonomous
     * @param turnDirectionInput 1=blue -1=red
     * @throws InterruptedException
     */
    boolean startNearRamp=false;  //Decides where starting position is
    boolean goToRamp=true;
    Servo dumpingBlock;


    public void runOpMode(int turnDirectionInput) throws InterruptedException {
        telemetry.addData("Init", "running");

        turnDirection = turnDirectionInput; //adjusts turns based on team color
        getRobotConfig();//Map Motors and Sensors
        //Configure and Reset
        runUsingEncoders();
        resetDriveEncoders();
        climberDumper.setPosition(0.5);
        sideArmL.setPosition(0.75);
        sideArmR.setPosition(0);
        doorR.setPosition(0.85);
        doorL.setPosition(0.15);
        beaconR.setPosition(0);
        beaconL.setPosition(1);
        debDumper.setPosition(0.5);
        dumpingBlock = hardwareMap.servo.get("dumper");
        sleep(500);
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
            if (gamepad1.x){ //if pressed gets out of way and avoids ramp
                goToRamp=false;
            }
            telemetry.addData("trigger beacon ", triggerBeacon);
            telemetry.addData("go to ramp ", goToRamp);
            telemetry.addData("Delay Seconds ", delay / 1000);
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
        if (goToRamp){
            telemetry.addData("going to", "ramp");
        }
        else {
            telemetry.addData("going to", "place next to ramp");
        }


        while (!adaFruitGyro.initDone) {
            adaFruitGyro.initIMU();
        }
        waitForStart();

        if (turnDirectionInput==-1){
            dumpingBlock.setPosition(0.35);
        } else {
            dumpingBlock.setPosition(0.55);
        }

        beaconR.setPosition(0.9);
        beaconL.setPosition(0.1);
        sleep(delay);
        collector.setPower(-0.7); //flaps backward to avoid getting cubes stuck
        climberDumper.setPosition(0.5); //makes sure climber dumper will not move
        if (startNearRamp) { //near ramp position
            drive(1600, .7, 0);
            pivot(36.5, 1, 0.3);
            drive(4500, 1, 0);
        } else { //far ramp position
            drive(1600, .7, 0);
            pivot(52.5, 1, 0.3);
            drive(6500, 1, 0);
        }
        newGyroTurn(42, 1);
        drive(2000, .2, 1); //drives to white line
        drive(60, -0.2, 0);
        newGyroTurn(90, 0.75);
       // stopMotors();
        collector.setPower(0); //kills colector
        driveUntilUltra(30, 0.1, 1200); //drives until 30 cm from wall
        if (triggerBeacon) { //goes to trigger beacon
            sleep(100);
            int leftred = cameraController.getLeftRed();//read image
            int rightred = cameraController.getRightRed();

            telemetry.addData("Colors", "Left " + leftred / 1000 + " Right: " + rightred / 1000);
            if (leftred > rightred){ //left side is red
                if (turnDirection == -1) { //on red team
                    beaconL.setPosition(0.7);
                } else { //on blue team
                    beaconR.setPosition(0.2);
                }
            } else { //right side is red
                if (turnDirection == -1) { //on red team
                    beaconR.setPosition(0.2);
                } else { //on blue team
                    beaconL.setPosition(0.7);
                }
            }
            sleep(100);
        }
        driveUntilUltra(15, 0.1, 200); //presses buttons
        waitForNextHardwareCycle();
        climberDumper.setPosition(0.65); //dumps climbers
        sleep(2000);
        climberDumper.setPosition(0.5);
        beaconR.setPosition(0.9);
        beaconL.setPosition(0.1);
        if (goToRamp) { //goes to ramp
            if (turnDirectionInput==1){
                sideArmR.setPosition(0.5);
            }
            else {
                sideArmL.setPosition(0.3);
            }
            drive(1000, -0.25, 0);
            pivot(200, 1, 2);
            collector.setPower(1);
            drive(2000, 1, 0);
            newGyroTurn(-45, 2);
            drive(5000, -1, 0);
        } else { //goes into place next to ramp
            newGyroTurn(180,2);
            drive(1500, 1, 0);
        }
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
