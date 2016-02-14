package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

/**
 * Created by August on 2/13/2016.
 */
public class AutonomousTesting extends AutonomousMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        getRobotConfig();
        beacon.setPosition(0.5); //left
        sleep(2000);
        beacon.setPosition(0.6); //right

while (true) {
    run_using_encoders();
    FR.setPower(1);
    sleep(1000);
    FR.setPower(0);


    FL.setPower(1);
    sleep(1000);
    FL.setPower(0);


    BR.setPower(1);
    sleep(1000);
    BR.setPower(0);


    BL.setPower(1);
    sleep(1000);
    BL.setPower(0);

    BL.setPower(1);
    FL.setPower(1);
    sleep(1000);
    BL.setPower(0);
    FL.setPower(0);

    sleep(1000);
    BR.setPower(1);
    FR.setPower(1);
    sleep(1000);
    BR.setPower(0);
    FR.setPower(0);

    sleep(5000);
    waitOneFullHardwareCycle();

}

    }
}
