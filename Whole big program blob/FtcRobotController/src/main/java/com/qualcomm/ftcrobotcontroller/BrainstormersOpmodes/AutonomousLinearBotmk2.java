package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;
/**
 * contains all the code to run Autonomous, it has no inherent side
 */
public abstract class AutonomousLinearBotmk2 extends AutonomousMethods {
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
        debDumper.setPosition((turnDirection + 1) / 2);
        startCam();


        sleep(5000);
        telemetry.addData("Init", "done");


        waitForStart(); //everything before this happens when you press init
        beacon.setPosition(1);


        //collector.setPower(-1);
        final boolean SLEEP = false;

        if (SLEEP) sleep(5000);
        drive(500, 1);
        telemetry.addData("starting", "turn");
        turnTo(37,0);
        drive(6500, 1);
        sleep(500);
        drive(1500, .15, false, false, 1);
        drive(820, .20, false, false, 0);
        turnTo(89,1);
        drive(400, -0.5, false, false,0);
        driveUntilUltra(35,0.15);
        //drive(500, 0.5);
        stopMotors();
        if(rightRed()>18000000)
            if(turnDirection==-1)
                beacon.setPosition(0.6);
            else
                beacon.setPosition(0.5);
        else
            if(turnDirection==-1)
                beacon.setPosition(0.5);
            else
                beacon.setPosition(0.6);

        sleep(1000);
        driveUntilUltra(18, 0.2);
        sleep(200);
        drive(500, -0.5);
        turnTo(89, 1);
        beacon.setPosition(1);
        sleep(500);
        drive(300, 0.5);
        climberDumperB.setPosition(1);
        sleep(1000);
        climberDumperB.setPosition(0);
        drive(500, -0.5);
        turnTo(-180, 1);
        drive(1000,1);
        turnTo(-135, 1);
        drive(1800, 1);
        turnTo(135, 1);
        collector.setPower(1);
        drive(1000,1);
        turnTo(-45, 1);
        drive(2000,-1);


        telemetry.addData("Red, Blue", " " + colorSensor2.blue() + " " + colorSensor2.red());
        sleep(100);
        waitOneFullHardwareCycle();
    }}