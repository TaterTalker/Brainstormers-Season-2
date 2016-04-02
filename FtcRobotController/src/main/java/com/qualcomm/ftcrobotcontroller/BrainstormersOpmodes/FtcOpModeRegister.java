/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

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

package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * determines which OpModes show uop on the control phone screen
 */
public class FtcOpModeRegister implements OpModeRegister {

    /**
     * The Op Mode Manager will call this method when it wants a list of all
     * available op modes. Add your op mode to the list to enable it.
     *
     * @param manager op mode manager
     */
    public void register(OpModeManager manager) {

        
        manager.register("Sensor Test", sensorTest.class);
        //manager.register("color test", ColorTest.class);
        manager.register("TeleOp Octopus Blue", TeleOpBlue.class);
        manager.register("TeleOp Octopus Red", TeleOpRed.class);
        manager.register("Auto-Blue Octopus", AutonomousBlueBotmk2.class);
        manager.register("Auto-Red Octopus", AutonomousRedBotmk2.class);
        manager.register("teleoptest" , TeleopTest.class);
        manager.register("autonomous test", AutonomousTest.class);
        manager.register("servo test", ServoOptimizer.class);
        //manager.register("Auto-Red Octopus", AutonomousRedBotmk2.class);
        //manager.register("why are we making changes at the competition this a bad idea", excusetomakechangesatthecompetition.class);
        //manager.register("ramp detection", rampDetection.class);
        //manager.register("autonomous test", ServoOptimizer.class);
        manager.register("new gyo test", AdafruitIMUTest.class);
        manager.register("autonmoose test", ServoOptimizer.class);
        manager.register("elephant teleop", ElephantTeleOp.class);
        manager.register("block detection test", CameraDebrisCountingTest.class);
        manager.register("wtf m8", jskdfkjsalkfdjlsjfdlskfdj.class);

    }
}
