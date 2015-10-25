package com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods;
import com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods.accelerometerReader;
/**
 * Created by August on 10/25/2015.
 */
public class rotTracker extends accelerometerReader{

    public static float degs=0;
    public static float rotSpeed;

    static Runnable rot_Tracker = new Runnable() {

        @Override
        public void run() {
            while(true) {
                rotSpeed += accelX;
                degs += rotSpeed;
            }
        }
    };

    static Thread rotTracker = new Thread(rot_Tracker);

    public static void startTracking(){
        rotTracker.start();
    }

}
