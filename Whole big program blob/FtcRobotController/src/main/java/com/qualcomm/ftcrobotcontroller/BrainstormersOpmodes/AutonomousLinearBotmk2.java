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
       // sideArmL.setPosition(0.05);
      //  sideArmR.setPosition(.75);
        doorL.setPosition(0.3);
        doorR.setPosition(0.8);
        debDumper.setPosition((turnDirection + 1) / 2);


        sleep(5000);
        telemetry.addData("Init", "done");
        startCam();

        waitForStart(); //everything before this happens when you press init


        //collector.setPower(-1);
        final boolean SLEEP = false;

        if (SLEEP) sleep(5000);;
        drive(500, 1);
        telemetry.addData("starting", "turn");
        turnTo(44);
        drive(6500,1);
        sleep(500);
        drive(1500, .25, false, false, 1);
        sleep(500);
        drive(345, .5);
        turnTo(90);
        sleep(500);
        stopMotors();
        sleep(2000);
        if(rightRed()>65000000)
            if(turnDirection==-1)
                beacon.setPosition(1);
            else
                beacon.setPosition(0);
        else
            if(turnDirection==-1)
                beacon.setPosition(0);
            else
                beacon.setPosition(1);

        drive(350, 0.5);
        stopMotors();
        climberDumperB.setPosition(1);
        sleep(1000);
        climberDumperB.setPosition(0);
        //  turn(-10 * turnDirection);

        telemetry.addData("Red, Blue", " " + colorSensor2.blue() + " " + colorSensor2.red());
        sleep(100);
        waitOneFullHardwareCycle();
    }}