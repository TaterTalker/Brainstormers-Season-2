package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;
import com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes.AdafruitIMUmanager;
/**
 * Created by August on 3/5/2016.
 */
public abstract class AdvancedMethods extends AutonomousMethods {
    void PIdrive(int distance, double power) throws InterruptedException {
        adafruitgyro.startIUM();
        resetEncoderDelta();
        final double deviationGain=.25; //how much deviation effects the robot
        double overShoot=0;
        double deviation=0;
        boolean hasReached=false;
        while(hasReached==false){
            telemetry.addData("encoder positions", " back right "+BRposition()+" back left "+BLposition());
            telemetry.addData("overShoot", overShoot);
            telemetry.addData("deviation", deviation);
            double yaw = adafruitgyro.getYaw();
            deviation=yaw*deviationGain;

            run_using_encoders();
            double leftPower=(-deviation)*Math.abs(power);
            double rightPower=(deviation)*Math.abs(power);
            leftPower+=power;
            rightPower+=power;
//            rightPower+=clip(1-(power+deviation+overShoot)*Math.abs(power), 0, 2);
//            leftPower-=clip(1-(power+deviation+overShoot)*Math.abs(power), 0, 2);
            setLeftPower(leftPower);
            setRightPower(rightPower);
            hasReached=Math.abs(BRposition()+BLposition())/2>Math.abs(distance);
            waitOneFullHardwareCycle();
        }
        stopMotors();
    }

    void newGyroTurn(double degrees, double tolerance) throws InterruptedException {
       adafruitgyro.startIUM(); //0s gyro
        final double gain=0.015;
        int  countwithintolerence = 0, count = 0;
        double heading,difference, cyclesMaxPower = 0;

        degrees *= turnDirection;
        double power;
        boolean rightTurn = false;
        run_using_encoders();

        while (true) { //while the turn hasn't been completed we run through this loop
            count++;
            heading = adafruitgyro.getYaw();
            difference = (degrees - heading) % 360; //calculates the angle based on where the robot is now and how far it has to go

            if (difference > 180) { //determines which way the robot will turn (left or right)
                difference = difference - 360.0;
            } else if (difference < -180) {
                difference = 360.0 + difference;
            }

            telemetry.addData("Heading", " " + heading + " " + difference + " " + rightTurn); //determines how fast the turn should be, as the turn gets greater the speed gets faster
            power = Math.abs(difference*gain);
            power = clip(power, 0.03, 0.35);

            if (cyclesMaxPower == 0) {
                cyclesMaxPower = Math.abs(difference / 30.0) + 3;
            }

            if (count < cyclesMaxPower && Math.abs(difference) > 20) { //speeds up out power to speed up turn during the beginning of the turn
                power = 1;
                telemetry.addData("1", "power");
            }

            telemetry.addData("power", "" + power);
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
