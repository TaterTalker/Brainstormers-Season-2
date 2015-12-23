package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by ethan on 12/22/2015.
 */
public class DriveStraightTest extends LinearOpMode {
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    GyroSensor gyroSensor;
    DcMotor FR;
    DcMotor collector;
    Servo climberDumper;
    Servo sideArmL;
    Servo lock;
    Servo sideArmR;
    boolean turnComplete = false;
    Servo debDumper;
    Servo door;
    UltrasonicSensor ultra1;
    UltrasonicSensor ultra2;
    int lastgyro;
    double turnChange = 1;
    int turnDirection = 1;
    private final double TURNRATIO = 18.3;
    private int loopCount = 0;
    private double ultrastate = 0;
    private boolean didEncodersReset = false;

    @Override
    public void runOpMode() throws InterruptedException {
        getRobotConfig();
        run_using_encoders();
        reset_drive_encoders();
        gyroSensor.calibrate();
        sleep(5000);
        waitForStart(); //everything before this happens when you press init
        driveStraight(5000, 1);

    }

    boolean encoders_have_reset() {
        if (didEncodersReset ||
                FL.getCurrentPosition() == 0 &&
                        FR.getCurrentPosition() == 0 &&
                        BL.getCurrentPosition() == 0 &&
                        BR.getCurrentPosition() == 0)
            didEncodersReset = true;
        return true;
    }

    void driveStraight(float distance, double speed) throws InterruptedException {
        reset_drive_encoders();
        resetGyro();
        // Start the drive wheel motors at full power
        while(!encoders_have_reset())
            sleep(1);

        while (!hasLeftReached(distance) && !hasRightReached(distance)) {
            double activeSpeed=speed;
            telemetry.addData("encoder values", "right:" + FR.getCurrentPosition() + " left:" + FL.getCurrentPosition());
            double turnheading = heading();
            if(turnheading>180)
                turnheading-=360;
            turnheading/=15;

            if(Math.abs(turnheading)>1)
                activeSpeed=clip(activeSpeed,-0.7,0.7);
            else if (turnheading!=0)
                activeSpeed=clip(activeSpeed,-0.9,0.9);

            telemetry.addData("heading ", "" + heading());
            run_using_encoders();
            setLeftPower(activeSpeed +turnheading);
            setRightPower(activeSpeed -turnheading);
        }
        stopMotors();
        reset_drive_encoders();
    }

    boolean hasLeftReached(double leftd) {

        return (Math.abs(FL.getCurrentPosition()) > leftd) &&
                (Math.abs(BL.getCurrentPosition()) > leftd);
    }

    boolean hasRightReached(double rightd) {

        return (Math.abs(FR.getCurrentPosition()) > rightd) &&
                (Math.abs(BR.getCurrentPosition()) > rightd);
    }
    double readFixedUltra(UltrasonicSensor sensor) {
        double val = 0;
        for (int i = 0; i < 10; i++) {
            val += sensor.getUltrasonicLevel();
        }
        val /= 10;
        return val;
    }

    void reset_drive_encoders() {

        FL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        FR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        BL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        BR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        didEncodersReset = false;
    }

    void run_using_encoders() {

        FL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        FR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    void getRobotConfig() {
        sideArmL = hardwareMap.servo.get("sideArmL");
        sideArmR = hardwareMap.servo.get("sideArmR");
        lock = hardwareMap.servo.get("lock");
        gyroSensor = hardwareMap.gyroSensor.get("G1");
        climberDumper = hardwareMap.servo.get("climberdumper");
        debDumper = hardwareMap.servo.get("debDumper");
        door = hardwareMap.servo.get("door");
        collector = hardwareMap.dcMotor.get("colmot");
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        ultra1 = hardwareMap.ultrasonicSensor.get("ultra1");
        ultra2 = hardwareMap.ultrasonicSensor.get("ultra2");
    }

    void setLeftPower(double power) {
        power = clip(power, -1, 1);
        run_using_encoders();
        FL.setPower(-power);
        BL.setPower(-power);
    }

    void setRightPower(double power) {
        power = clip(power, -1, 1);
        run_using_encoders();
        FR.setPower(power);
        BR.setPower(power);
    }

    int heading(){
        int head;
        head=gyroSensor.getHeading()-lastgyro;
        if (head<0)
            head+=360;
        return (head);
    }

    void stopMotors(){
        while(FR.isBusy()==true ||
                BR.isBusy()==true||
                FL.isBusy()==true||
                BL.isBusy()==true){
            run_using_encoders();
            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
        }
    }

    double clip(double variable, double min, double max) {
        if (variable < min)
            variable = min;

        if (variable > max)
            variable = max;

        return variable;
    }
    void resetGyro() {
        while(heading()!=0) {
            lastgyro = gyroSensor.getHeading();
        }
    }

}
