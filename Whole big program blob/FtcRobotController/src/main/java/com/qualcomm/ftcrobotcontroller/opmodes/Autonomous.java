package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.Drive;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by August on 10/10/2015.
 */
public class Autonomous extends OpMode {
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    DcMotor FR;
    Servo climberDumper;

    private int v_state = 0;

    //Map the motors.
    public void init() {
        climberDumper = hardwareMap.servo.get("climberdumper");
        v_state= 0;
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
    }

    //Start the program and reset the encoders.
    @Override
    public void start() {

        super.start();
        v_state = 0;
    }

    //Loop through the state machine completing each task.
    public void loop() {

        switch (v_state) {

            //Reset Motors.
            case -1:
                //null state
                //to be used if action should only be performed once
                break;

            case 0:
                reset_drive_encoders();
                run_using_encoders();
                v_state++;
                break;

            case 1:
                turn(310, 0.2);
                telemetry.addData("rightTicks", "" + FL.getCurrentPosition());
                telemetry.addData("leftTicks", "" + FR.getCurrentPosition());
                break;

            case 2:
                drive(5000,1);
                break;
            case 3:
                climberDumper.setPosition(0);
            default:

                break;
        }
        telemetry.addData("Text", "State: " + v_state);
    }

    //The drive function.


    void drive(float distance, float speed) {
    // Start the drive wheel motors at full power

        if (hasLeftReached(distance) && hasRightReached(distance)) {
            setLeftPower(0);
            setRightPower(0);
            v_state++;
        }

        else {
            setLeftPower(speed);
            setRightPower(speed);
        }
    }

    void reset_drive_encoders() {

        FL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        FR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        BL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        BR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    void run_using_encoders() {

        FL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        FR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    boolean hasLeftReached(double leftd) {

        return (Math.abs(FL.getCurrentPosition()) > leftd) &&
                (Math.abs(BL.getCurrentPosition()) > leftd);
    }

    boolean hasRightReached(double rightd) {

        return (Math.abs(FR.getCurrentPosition()) > rightd) &&
                (Math.abs(BR.getCurrentPosition()) > rightd);
    }

    boolean have_drive_encoders_reset() {

        return (FL.getCurrentPosition() == 0 &&
                FR.getCurrentPosition() == 0 &&
                BL.getCurrentPosition() == 0 &&
                BR.getCurrentPosition() == 0);
    }

    void setLeftPower(double power) {
        power=clip(power,-1,1);
        FL.setPower(-power);
        BL.setPower(-power);
    }

    void setRightPower(double power) {
        power=clip(power,-1,1);
        FR.setPower(power);
        BR.setPower(power);
    }

    double clip(double variable, double min, double max){
        if(variable<min)
            variable=min;

        if (variable>max)
            variable=max;

        return variable;
    }

    void initEncoders(){
        reset_drive_encoders();
        run_using_encoders();
    }

    void turn(int degrees, double power){
        if(hasLeftReached(degrees*10)&&hasRightReached(degrees*10)){
            setLeftPower(0);
            setRightPower(0);
            reset_drive_encoders();
            run_using_encoders();
            v_state++;
        }
        else{
            setLeftPower(power);
            setRightPower(-power);
        }
    }
}
