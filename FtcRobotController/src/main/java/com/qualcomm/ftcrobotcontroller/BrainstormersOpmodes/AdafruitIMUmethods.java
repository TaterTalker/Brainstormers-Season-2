package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;

/**
 * Created by Owner on 8/31/2015.
 */
public  class AdafruitIMUmethods{

    LinearOpMode opmode;

    public  AdafruitIMUmethods(LinearOpMode opmode)  {
        this.opmode = opmode;
    }

    AdafruitIMU boschBNO055;

    //The following arrays contain both the Euler angles reported by the IMU (indices = 0) AND the
    // Tait-Bryan angles calculated from the 4 components of the quaternion vector (indices = 1)
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

    long systemTime;//Relevant values of System.nanoTime

    /************************************************************************************************
     * The following method was introduced in the 3 August 2015 FTC SDK beta release and it runs
     * before "start" runs.
     */
    public void initIMU() {
        opmode.telemetry.addData("1","");
        systemTime = System.nanoTime();
        try {
            opmode.telemetry.addData("2","");
            boschBNO055 = new AdafruitIMU(opmode.hardwareMap, "bno055"


                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte)AdafruitIMU.OPERATION_MODE_IMU);
                    boschBNO055.startIMU();
            opmode.telemetry.addData("passed", "passed");
        } catch (RobotCoreException e){
            Log.i("FtcRobotController", "Exception: " + e.getMessage());
            opmode.telemetry.addData("3", "");
        }
        Log.i("FtcRobotController", "IMU Init method finished in: "
                + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
        //ADDRESS_B is the "standard" I2C bus address for the Bosch BNO055 (IMU data sheet, p. 90).
        //BUT DAVID PIERCE, MENTOR OF TEAM 8886, HAS EXAMINED THE SCHEMATIC FOR THE ADAFRUIT BOARD ON
        //WHICH THE IMU CHIP IS MOUNTED. SINCE THE SCHEMATIC SHOWS THAT THE COM3 PIN IS PULLED LOW,
        //ADDRESS_A IS THE IMU'S OPERATIVE I2C BUS ADDRESS
        //IMU is an appropriate operational mode for FTC competitions. (See the IMU datasheet, Table
        // 3-3, p.20 and Table 3-5, p.21.)
        opmode.telemetry.addData("gotit", "");
    }

    /************************************************************************************************
     * Code to run when the op mode is first enabled goes here
     * @see OpMode#start()
     */
    public void startIUM() {
        opmode.telemetry.addData("4","");

        /*
      	* Use the hardwareMap to get the dc motors, servos and other sensors by name. Note
      	* that the names of the devices must match the names used when you
      	* configured your robot and created the configuration file. The hardware map
      	* for this OpMode is not initialized until the OpModeManager's "startActiveOpMode" method
      	* runs.
    		*/
        systemTime = System.nanoTime();
        boschBNO055.startIMU();//Set up the IMU as needed for a continual stream of I2C reads.
        Log.i("FtcRobotController", "IMU Start method finished in: "
                + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
    }

    /***********************************************************************************************
     * This method will be called repeatedly in a loop
     * @see OpMode#loop()
     * NOTE: BECAUSE THIS "loop" METHOD IS PART OF THE OVERALL OpMode/EventLoop/ReadWriteRunnable
     * MECHANISM, ALL THAT THIS METHOD WILL BE USED FOR, IN AUTONOMOUS MODE, IS TO:
     * 1. READ SENSORS AND ENCODERS AND STORE THEIR VALUES IN SHARED VARIABLES
     * 2. WRITE MOTOR POWER AND CONTROL VALUES STORED IN SHARED VARIABLES BY "WORKER" THREADS, AND
     * 3. SEND TELELMETRY DATA TO THE DRIVER STATION
     * THIS "loop" METHOD IS THE ONLY ONE THAT "TOUCHES" ANY SENSOR OR MOTOR HARDWARE.
     */

    public double getYaw(){ //READ THIS IGNORE THE REST OF THIS FILE
        boschBNO055.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[1]*-1.0; //needs * -1 b/c gyro is upside down (thx rob)
    }
}