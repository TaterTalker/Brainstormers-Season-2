package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.util.Log;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.ByteArrayOutputStream;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public abstract class CameraOp extends LinearOpMode {
    private Camera camera;
    public CameraPreview preview;
    public Bitmap image;
    private int width;
    private int height;
    private YuvImage yuvImage = null;
    private int looped = 0;
    private String data;
    public int color;

    private int red(int pixel) {
        return (pixel >> 16) & 0xff;
    }

    private int green(int pixel) {
        return (pixel >> 8) & 0xff;
    }

    private int blue(int pixel) {
        return pixel & 0xff;
    }

    private Camera.PreviewCallback previewCallback = new Camera.PreviewCallback() {
        public void onPreviewFrame(byte[] data, Camera camera) {
            Camera.Parameters parameters = camera.getParameters();
            width = parameters.getPreviewSize().width;
            height = parameters.getPreviewSize().height;
            yuvImage = new YuvImage(data, ImageFormat.NV21, width, height, null);
            looped += 1;
        }
    };

    private void convertImage() {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new Rect(0, 0, width, height), 0, out);
        byte[] imageBytes = out.toByteArray();
        image = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length);
    }

    public void startCam() {
        camera = ((FtcRobotControllerActivity) hardwareMap.appContext).camera;
        camera.setPreviewCallback(previewCallback);
        Camera.Parameters parameters = camera.getParameters();
        data = parameters.flatten();

        ((FtcRobotControllerActivity) hardwareMap.appContext).initPreview(camera, this, previewCallback);
    }

    int[] pixel(int x, int y) {
        int pixelTarg = image.getPixel(x, y);
        int[] tmpArray = {red(pixelTarg), green(pixelTarg), blue(pixelTarg)};
        return tmpArray;
    }

    public int leftRed() {
        convertImage();
        int value = 0;
        for (int x = 0; x < width / 2; x+= 4) {
            for (int y = 0; y < height-4; y+=4) {
                value += pixel(x, y)[0];
            }
        }
        return value;
    }

    public int rightRed() {
        convertImage();
        int value = 0;
        for (int x = width - 1; x > width / 2; x -= 4) {
            for (int y = 0; y < height - 4; y = y + 4) {
                value += pixel(x, y)[0];
            }
        }
        return value;
    }
}
