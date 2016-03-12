package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;
/**
 * contains all the code to run Autonomous, it has no inherent side
 */
public abstract class AutonomousLinearBotmk2 extends AvancedMethods {
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
        run_using_encoders();
        reset_drive_encoders();
      //  gyroSensor.calibrate();
        climberDumperB.setPosition(0);
        armAngle1.setPosition(0.5);
        armAngle2.setPosition(0.5);
        sideArmL.setPosition(0.75);
        sideArmR.setPosition(0);
        doorL.setPosition(0.3);
        doorR.setPosition(0.8);
        beacon.setPosition(0);
        debDumper.setPosition((turnDirection + 1) / 2);
        sleep(5000);
        telemetry.addData("Init", "done");
        boolean tigBacon=true;

        while (!gamepad1.a && !gamepad1.b) {//adds in delay from button press
            if(gamepad1.y==true)
                tigBacon=false;
            if (gamepad1.dpad_up)
                delay = delay + 1000;
            else if (gamepad1.dpad_down)
                delay = delay - 1000;
            telemetry.addData("Delay Seconds:", delay / 1000);
            sleep(250);
        }

        if (gamepad1.a)  //sets starting position of robot
            startNearRamp=true;

        if (startNearRamp)
            telemetry.addData("Near Ramp","");
        else
            telemetry.addData("Far From Ramp", "");
        telemetry.addData("Ready", "");
        if(tigBacon){
            telemetry.addData("Beacon" , "Activated");
        }
        else{
            telemetry.addData(" No Beacon" , "Deactivated");
        }

        waitForStart(); //everything before this happens when you press init

        beacon.setPosition(1);
        sleep(300);
        sleep(delay);
        collector.setPower(-0.7);
        climberDumperB.setPosition(0.1);
        if (startNearRamp) { //near ramp position
            drive(2000, .4, false, false, 0);
            if (turnDirectionInput == 1) {
                turnTo(36, 0);
            } else {
                turnTo(37, 0);
            }
            drive(4600, 1, true, true, 0);
        }
        else { //far ramp position
            drive(1600, .4, false, false, 0);
            if (turnDirectionInput == 1) {
                turnTo(50, 0);
            } else {
                turnTo(51, 0);
            }
            drive(7000, 1, true, true, 0);
        }
        startCam();
        turnTo(25, 1);
        drive(873, .15, false, false, 1);
        drive(700, .20, false, false, 0);
        if (turnDirectionInput == 1){
            turnTo(88, 0);
        }
        else{
            turnTo(86,0);
        }
        stopMotors();
        collector.setPower(0);
        driveUntilUltra(30, 0.1, 600);


        //use camera to analyze the image and get the left and right red values
        if (tigBacon==true) {
            sleep (700);
            int leftred = leftRed();
            int rightred = rightRed();

            telemetry.addData("Colors", "Left " + leftred / 1000 + " Right: " + rightred / 1000);
            if (leftred > rightred) //left side is red
                if (turnDirection == -1)
                    beacon.setPosition(0.3);
                else
                    beacon.setPosition(0.7);
            else //right side is red
                if (turnDirection == -1)
                    beacon.setPosition(0.7);
                else
                    beacon.setPosition(0.3);
            sleep(500);
        }
        driveUntilUltra(15, 0.1, 200);
        waitForNextHardwareCycle();
        driveUntilUltra(15, 0.1, 200);
        drive(65, -0.2, false, false, 0);
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
        telemetry.addData("Red, Blue", " " + colorSensor2.blue() + " " + colorSensor2.red());
        sleep(100);
        waitOneFullHardwareCycle();
        */
    }

    //separate turn test function not in use.
    public void turnTest() throws InterruptedException {
        long start= System.currentTimeMillis();
        newturnTo(36, 0.5);
        newturnTo(25, 0.5);
        newturnTo(88, 0.5);
        newturnTo(-170, 0.5);
        newturnTo(135, 0.5);
        newturnTo(-52, 0.5);
        long end=System.currentTimeMillis();
        telemetry.addData("timing", Long.toString(end-start));
        sleep(150000);
    }
}
