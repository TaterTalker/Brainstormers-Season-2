package com.qualcomm.ftcrobotcontroller.NewStructure;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by ethan on 4/8/2016.
 */
public abstract class NewAutonomous extends LinearOpMode {

    AutoBot autoBot;

    boolean triggerBeacon = true;

    boolean startNearRamp=false;  //Decides where starting position is
    boolean goToRamp=true;
    int delay = 0;

    public void runOpMode(int side) throws InterruptedException {
        autoBot = new AutoBot(side, this);

        telemetry.addData("finished", "finished it");

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


        while (!autoBot.adaFruitGyro.initDone) {
            autoBot.adaFruitGyro.initIMU();
        }

        waitForStart();
        autoBot.start();
        sleep(100+ delay);
        if (startNearRamp) { //near ramp position
            autoBot.drive(1600, .7, 0);
            autoBot.pivot(38.5, 1, 0.5);
            autoBot.drive(4500, 1, 0);
        } else { //far ramp position
            autoBot.drive(1600, .7, 0);
            autoBot.pivot(52.5, 1, 0.5);
            autoBot.drive(6500, 1, 0);
        }
        //autoBot.newGyroTurn();(42, 1);
        autoBot.drive(2000, .2, 1); //drives to white line
        autoBot.drive(60, -0.2, 0);
        autoBot.newGyroTurn(90, 2);
        // stopMotors();
        //debrisCounter.interrupt();
        autoBot.blockCounterThread.interrupt();
        autoBot.setCollectorDirection(0);
        autoBot.collector.setPower(0);
        autoBot.cameraController.startBackCam();
        autoBot.beaconR.setPosition(0);
        autoBot.beaconL.setPosition(1);
        autoBot.driveUntilUltra(15, 0.1, 1200); //drives until 15 cm from wall
        autoBot.drive(120, .1,0);
        autoBot.climberDumper.setPosition(1); //dumps climbers
        sleep(2000);
        autoBot.climberDumper.setPosition(0.4);
        telemetry.addData("before drive after climbers", "");
        autoBot.drive(500, -0.1, 0);
        autoBot.climberDumper.setPosition(0.5);
        telemetry.addData("after drive before beacon", "");
        if (triggerBeacon) { //goes to trigger beacon
            sleep(100);
            telemetry.addData("before camera call","");
            int leftred = autoBot.cameraController.getLeftRed();//read image
            telemetry.addData("after left camera call", "");
            int rightred = autoBot.cameraController.getRightRed();

            telemetry.addData("Colors", "Left " + leftred / 1000 + " Right: " + rightred / 1000);
            if (leftred > rightred){ //left side is red
                if (side == -1) { //on red team
                    autoBot.beaconL.setPosition(0.6);
                } else { //on blue team
                    autoBot.beaconR.setPosition(0.3);
                }
            } else { //right side is red
                if (side == -1) { //on red team
                    autoBot.beaconR.setPosition(0.3);
                } else { //on blue team
                    autoBot.beaconL.setPosition(0.6);
                }
            }
            sleep(100);
        }
        telemetry.addData("beacon check", "");
        autoBot.driveUntilUltra(15, 0.1, 200); //presses buttons
        autoBot.drive(50, 0.2, 0);
        autoBot.drive(80, -0.2, 0);

        autoBot.beaconR.setPosition(.7);
        autoBot.beaconL.setPosition(.2);
        //autoBot.drive(40, 0.2,0);
        if (goToRamp) { //goes to ramp
            if (side==1){
                autoBot.sideArms.setSideArmRpos(0.5f);
            }
            else {
                autoBot.sideArms.setSideArmLpos(0.3f);
            }
            autoBot.drive(1000, -0.25, 0);
            autoBot.pivot(200, 1, 2);
            autoBot.setCollectorDirection(-1);
            autoBot.collector.setPower(-1);
            autoBot.drive(2200, 1, 0);
            autoBot.newGyroTurn(-45, 2);


            autoBot.arm.setArmPower(-1);
            autoBot.drive(1000, -1,0);
            autoBot.arm.setArmPower(0);
            if(side == -1){
                autoBot.sideArms.setSideArmLpos(0);
                autoBot.sideArms.setSideArmRpos(0.05f);
            }
            else{
                autoBot.sideArms.setSideArmRpos(1);
                autoBot.sideArms.setSideArmLpos(0.8f);
            }
            autoBot.drive(5000, -1, 0);
        } else { //goes into place next to ramp
            autoBot.newGyroTurn(180,2);
            autoBot.drive(1500, 1, 0);
        }

    }
}


