package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;
import com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes.AdafruitIMUmanager;
/**
 * Created by August on 3/5/2016.
 */
public abstract class advancedMethods extends AutonomousMethods {
    void PIdrive(int distance, double power){
        int deviationGain=1;
        int overShoot=0;
        boolean hasReached=Math.abs(FRposition()+FLposition()+BRposition()+BLposition())/4<Math.abs(distance);

        while(hasReached){
            int deviation=FRposition()+BRposition()-FLposition()-BLposition();

        }
    }

    void newturnTo(int degrees, int tolerance) throws InterruptedException {
        int  countwithintolerence = 0, count = 0;
        double heading,difference, cyclesMaxPower = 0;

        degrees *= turnDirection;
        double power;
        boolean rightTurn = false;
        run_using_encoders();

        while (true) { //while the turn hasn't been completed we run through this loop
            count++;
            heading = advancedgyro.getYaw();
            difference = (degrees - heading) % 360; //calculates the angle based on where the robot is now and how far it has to go

            if (difference > 180) { //determines which way the robot will turn (left or right)
                difference = difference - 360;
            } else if (difference < -180) {
                difference = 360 + difference;
            }

            telemetry.addData("Heading", " " + heading + " " + difference + " " + rightTurn); //determines how fast the turn should be, as the turn gets greater the speed gets faster
            power = Math.abs(difference / 45);
            power = clip(power, 0.075, 0.35);

            if (cyclesMaxPower == 0) {
                cyclesMaxPower = Math.abs(difference / 30) + 3;
            }

            if (count < cyclesMaxPower && Math.abs(difference) > 20) { //speeds up out power to speed up turn during the beginning of the turn
                power = 1;
            }

            if (Math.abs(difference) <= tolerance) { //how far off the turn can be while still being successful (tolerance of turn)
                countwithintolerence++;
                if (countwithintolerence > 15) {
                    break;
                }
                setLeftPower(0);
                setRightPower(0);
            } else if (difference > 0) {
                countwithintolerence = 0;
                setLeftPower(power);
                setRightPower(-power);
                rightTurn = true;
            } else {
                countwithintolerence = 0;
                setLeftPower(-power);
                setRightPower(power);
                rightTurn = false;
            }

            waitOneFullHardwareCycle();
        }

        telemetry.addData("Do", "ne");
        setLeftPower(0);
        setRightPower(0);
        sleep(20);
        setLeftPower(0);
        setRightPower(0);
    }


}
