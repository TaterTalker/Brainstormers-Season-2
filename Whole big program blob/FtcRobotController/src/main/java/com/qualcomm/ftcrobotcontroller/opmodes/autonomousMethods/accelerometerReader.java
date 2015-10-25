package com.qualcomm.ftcrobotcontroller.opmodes.autonomousMethods;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;

/**
 * Created by August on 10/25/2015.
 */
public class accelerometerReader extends Activity implements SensorEventListener {

    public static float accelX;
    public float accelY;
    public float accelZ;

    public static void turn(float degrees, float speed){

    }

    public void onSensorChanged(SensorEvent event) {
        Sensor mySensor = event.sensor;

        if (mySensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            accelX = event.values[0];
            accelY = event.values[1];
            accelZ = event.values[2];
        }
    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }
}
