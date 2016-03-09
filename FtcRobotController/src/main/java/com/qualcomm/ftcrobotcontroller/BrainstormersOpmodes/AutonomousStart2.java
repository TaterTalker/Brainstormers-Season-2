package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;
/**
 * contains all the code to run Autonomous, it has no inherent side
 */
public abstract class AutonomousStart2 extends AutonomousMethods {
    /**
     * run all other functions to perform autonomous
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
        sideArmL.setPosition(0.75);
        sideArmR.setPosition(0);
        doorL.setPosition(0.3);
        doorR.setPosition(0.8);
        beacon.setPosition(0);
        debDumper.setPosition((turnDirection + 1) / 2);
        startCam();
        sleep(5000);
        telemetry.addData("Init", "done");
        while (!gamepad1.a) {
            if (gamepad1.dpad_up)
                delay = delay + 1000;
            else if (gamepad1.dpad_down)
                delay = delay - 1000;
            telemetry.addData("Delay Seconds:", delay / 1000);
            sleep(250);
        }
        telemetry.addData("Ready","");




        waitForStart(); //everything before this happens when you press init
        beacon.setPosition(1);
        sleep(300);
        sleep(delay);
        collector.setPower(-0.7);

        //collector.setPower(-1);
        drive(2000, .4, false, false, 0);
        if (turnDirectionInput == 1){
            turnTo(40, 0);
        }
        else{
            turnTo(42,0);
        }
        drive(5000, 1, true, true, 0);
        turnTo(25, 1);
        drive(900, .15, false, false, 1);
        //old value 725
        drive(700, .20, false, false, 0);
        if (turnDirectionInput == 1){
            turnTo(88, 0);
        }
        else{
            turnTo(86,0);
        }

        //  drive(350, -0.2, false, false, 0);
        // driveUntilUltra(35, 0.15);
        //drive(500, 0.5);
        stopMotors();
        collector.setPower(0);
        driveUntilUltra(30, 0.1, 500);
        sleep (700);
        int leftred = leftRed();
        int rightred = rightRed();
        telemetry.addData("Colors", "Left " + leftred/1000 + " Right: " + rightred/1000);
        //Added Sleep  to look at values
        if(leftred>rightred)
            if(turnDirection==-1)
                beacon.setPosition(0.1);
            else
                beacon.setPosition(0.5);
        else
        if(turnDirection==-1)
            beacon.setPosition(0.5);
        else
            beacon.setPosition(0.1);

        sleep(500);
        driveUntilUltra(15, 0.1, 500);
        climberDumperB.setPosition(1);
        sleep(1000);
        climberDumperB.setPosition(0);
        drive(500, -0.25, false, false, 0);
        beacon.setPosition(0.9);
        turnTo(-170, 1);
        collector.setPower(1);
        drive(3000, 1, false, false, 0);
        collector.setPower(1);
        turnTo(135, 1);
        //drive(700, -1, false, false, 0);

        if (turnDirectionInput == 1) {
            turnTo(-52, 1);
        }
        else{
            turnTo(-55, 1);
        }

        //up positions
        //sideArmL.setPosition(0.75);
        //sideArmR.setPosition(0);


        if (turnDirectionInput ==1){
            sideArmR.setPosition(0.6);
        }
        else {
            sideArmL.setPosition(0.1);
        }

        drive(10000, -0.7, false, false, 0);


        telemetry.addData("Red, Blue", " " + colorSensor2.blue() + " " + colorSensor2.red());
        sleep(100);
        waitOneFullHardwareCycle();
    }
    public void turnTest() throws InterruptedException {
        long start= System.currentTimeMillis();
        turnTo(36, 0);
        turnTo(25, 1);
        turnTo(88, 0);
        turnTo(-170, 1);
        turnTo(135, 1);
        turnTo(-52, 1);
        long end=System.currentTimeMillis();
        telemetry.addData("timing", Long.toString(end-start));
        sleep(150000);
    }
}

