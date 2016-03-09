package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

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
}
