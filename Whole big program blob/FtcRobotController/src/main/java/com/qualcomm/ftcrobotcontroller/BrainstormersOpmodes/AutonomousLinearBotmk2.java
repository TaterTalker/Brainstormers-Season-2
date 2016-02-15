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



        if (SLEEP) sleep(500000);

//        //test program
//        turnTo(90, 1);
//        sleep(100);
//        turnTo(-90, 1);
//        sleep(100);
//        turnTo (150, 1);
//        sleep(10000);

        drive(2000, .25, false, false, 0);
        sleep(500);
        telemetry.addData("starting", "turn");
        turnTo(37, 0);
        drive(4500, 1, true, true, 0);
        sleep(500);
        drive(3500, .15, false, false, 1);
        drive(725, .20, false, false, 0);
        turnTo(88, 0);
        drive(350, -0.2, false, false, 0);
        driveUntilUltra(35, 0.15);
        //drive(500, 0.5);
        stopMotors();
        if(rightRed()>18000000)
            if(turnDirect   ion==-1)
                beacon.setPosition(0.6);
            else
                beacon.setPosition(0.5);
        else
            if(turnDirection==-1)
                beacon.setPosition(0.5);
            else
                beacon.setPosition(0.6);

        sleep(1000);
        driveUntilUltra(23, 0.2);
        climberDumperB.setPosition(1);
        sleep(500);
        climberDumperB.setPosition(0);
        drive(500, -0.25, false, false, 0);
        turnTo(-170, 1);
        drive(3000, 1, false, false, 0);
        collector.setPower(1);
        turnTo(135, 1);
        drive(500, -1, false, false, 0);
        turnTo(-49, 1);
        drive(10000, -1, false, false, 0);



        telemetry.addData("Red, Blue", " " + colorSensor2.blue() + " " + colorSensor2.red());
        sleep(100);
        waitOneFullHardwareCycle();
    }}