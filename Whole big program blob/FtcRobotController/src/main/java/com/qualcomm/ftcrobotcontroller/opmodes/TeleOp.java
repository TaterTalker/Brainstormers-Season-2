/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.ftcrobotcontroller.opmodes.mainDriving;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TeleOp extends OpMode {
	DcMotor FL;
	DcMotor BR;
	DcMotor BL;
	DcMotor FR;
	DcMotor collector;
	DcMotor extendor1;
	DcMotor extendor2;
	DcMotor lock;
	float YPower, XPower, rotPower;
	int direction=1;
	int directionOld=1;

	/**
	 * Constructor
	 */
	public TeleOp() {

	}

	/*
	 * Code to run when the op mode is initialized goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
	 */
	@Override
	public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */
		
		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *   
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
		FR = hardwareMap.dcMotor.get("FR");
		FL = hardwareMap.dcMotor.get("FL");
		BR = hardwareMap.dcMotor.get("BR");
		BL = hardwareMap.dcMotor.get("BL");
		collector = hardwareMap.dcMotor.get("colmot");
		extendor1 = hardwareMap.dcMotor.get("ext1");
		extendor2 = hardwareMap.dcMotor.get("ext2");
		lock = hardwareMap.dcMotor.get("lock");
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {
		if(gamepad1.x==true&&direction==directionOld){
			direction*=-1;
		}
		directionOld=direction;
		drive();
		atatchmentControl();
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Forwards power",  "" + String.format("%.2f", YPower));
        telemetry.addData("Left/Right Power", "" + String.format("%.2f", XPower));
		telemetry.addData("Rotation power", "" + String.format("%.2f", rotPower));
		telemetry.addData("Driving Direction", "" + String.format("%s", direction));

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 *
	 *
	 *
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}

	private void drive() {
		float YVal = direction*gamepad1.left_stick_y;
		float XVal = direction*gamepad1.left_stick_x;
		float RotVal = -direction*gamepad1.right_stick_x;
		// clip the right/left values so that the values never exceed +/- 1
		YPower = Range.clip(YVal, -1, 1);
		XPower = Range.clip(XVal, -1, 1);
		rotPower = Range.clip(RotVal, -1, 1);

		float FRpower = YPower+XPower-rotPower;
		float FLpower = -(YPower-XPower+rotPower);
		float BRpower = YPower-XPower-rotPower;
		float BLpower = -(YPower+XPower+rotPower);

		FRpower = Range.clip(FRpower, -1, 1);
		FLpower = Range.clip(FLpower, -1, 1);
		BRpower = Range.clip(BRpower, -1, 1);
		BLpower = Range.clip(BLpower, -1, 1);

		// write the values to the motors
		FR.setPower(FRpower);
		BR.setPower(FLpower);
		FL.setPower(BRpower);
		BL.setPower(BLpower);
	}


	private void atatchmentControl() {

		int collectorval;
		if (gamepad2.dpad_up == true) {
			collectorval=1;
		}
		else if(gamepad2.dpad_down == true){
			collectorval=-1;
		}
		else collectorval=0;
		collector.setPower(collectorval);


	}

}
