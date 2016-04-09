package com.qualcomm.ftcrobotcontroller.NewStructure.Parts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by ethan on 4/6/2016.
 */
public class WheelBase {
    OpMode opMode;
    DcMotor fl;
    DcMotor fr;
    DcMotor br;
    DcMotor bl;

    int fRold;
    int bRold;
    int fLold;
    int bLold;

    float frPower;
    float brPower;
    float flPower;
    float blPower;

    float yPower;
    float xPower;

    float driveMod = 1;


    public WheelBase(OpMode varopmode){

        opMode = varopmode;

        fr = opMode.hardwareMap.dcMotor.get("fr");
        fl = opMode.hardwareMap.dcMotor.get("fl");
        br = opMode.hardwareMap.dcMotor.get("br");
        bl = opMode.hardwareMap.dcMotor.get("bl");

    }


    public DcMotor getFr(){
        return fr;
    }


    public void driveBackwards(){
        fr.setPower(1);
        br.setPower(1);
        fl.setPower(-1);
        bl.setPower(-1);
    }

    public void setInput(float x, float y){

        yPower = Range.clip(y, -1, 1);
        xPower = Range.clip(x, -1, 1);

        /**
         * combines the rotation and speed together
         */
        frPower = yPower + xPower;
        brPower = yPower + xPower;
        flPower = -yPower + xPower;
        blPower = -yPower + xPower;

        if (!opMode.gamepad1.y){
            fr.setPower(Range.clip(frPower, -1, 1) * driveMod);
            br.setPower(Range.clip(brPower, -1, 1) * driveMod);
            fl.setPower(Range.clip(flPower, -1, 1) * driveMod);
            bl.setPower(Range.clip(blPower, -1, 1) * driveMod);
        }

    }

    public void setDriveMod(float driveMod){
        this.driveMod = driveMod;

    }

   public void resetDriveEncoders() {

        fl.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        fr.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        bl.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        br.setMode(DcMotorController.RunMode.RESET_ENCODERS);


    }

    /**
     * sets all drive encoders to run using encoders mode
     */
    //Run using Encoders.
   public void runUsingEncoders() {

        fl.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        fr.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        bl.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        br.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }
    public void setLeftPower(double power) {
        power = Range.clip(power, -1, 1);
        runUsingEncoders();
        fl.setPower(power);
        bl.setPower(power);
    }

   public void setRightPower(double power) {
        power = Range.clip(power, -1, 1); //because david is trash at building robots and it turns by itself we need to slow this side down, you know it would be so much more effective if you did it in hardware you ass - August
        runUsingEncoders();
        fr.setPower(-power);
        br.setPower(-power);
    }

    public void superstopMotors() throws InterruptedException {
        int iterations = 0;
        while ((fr.isBusy() || br.isBusy() || fl.isBusy() || bl.isBusy()) && iterations < 10) {
            iterations++;
            opMode.telemetry.addData("motor status", fr.isBusy() + " " + br.isBusy() + " " + fl.isBusy() + " " + bl.isBusy());
            runUsingEncoders();
            br.setPower(0);
            bl.setPower(0);
            fl.setPower(0);
            fr.setPower(0);

        }

    opMode.telemetry.addData("stopping", "complete");
}
    public void stopMotors(){
        br.setPower(0);
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
    }

    public void resetEncoderDelta() {
        fRold = fr.getCurrentPosition();
        bRold = br.getCurrentPosition();
        fLold = fl.getCurrentPosition();
        bLold = bl.getCurrentPosition();
    }
   public int brPosition() {
        return (br.getCurrentPosition() - bRold);
    }

    /**
     * gets adjusted position of the front left motor
     * reset to 0 using {@link #resetEncoderDelta()}
     *
     * @return the adjusted value
     */
    public int fLPosition() {
        return (fLold - fl.getCurrentPosition());
    }

    /**
     * gets adjusted position of the back left motor
     * reset to 0 using {@link #resetEncoderDelta()}
     *
     * @return the adjusted value
     */
    public int blPosition() {
        return (bLold - bl.getCurrentPosition());
    }

}
