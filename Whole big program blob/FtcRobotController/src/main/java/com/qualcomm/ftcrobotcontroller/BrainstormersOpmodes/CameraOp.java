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
    /**
     * the bitmap
     */
    public Bitmap image;
    /**
     * width of the image
     */
    private int width;
    /**
     * height of the image
     */
    private int height;
    private YuvImage yuvImage = null;
    private int looped = 0;
    private String data;
    public int color;

    /**
     * gets the red value of a pixel
     * @param pixel the target pixel
     * @return the pixel's red value
     */
    private int red(int pixel) {
        return (pixel >> 16) & 0xff;
    }

    /**
     * gets the green value of a pixel
     * @param pixel the target pixel
     * @return the pixel's green value
     */
    private int green(int pixel) {
        return (pixel >> 8) & 0xff;
    }

    /**
     * gets the blue value of a pixel
     * @param pixel the target pixel
     * @return the pixel's blue value
     */
    private int blue(int pixel) {
        return pixel & 0xff;
    }

    /**
     * reads the camera. It then exports the image dimentions and creates a yuv image.
     * @see #width
     * @see #height
     * @see #yuvImage
     */
    private Camera.PreviewCallback previewCallback = new Camera.PreviewCallback() {
        public void onPreviewFrame(byte[] data, Camera camera) {
            Camera.Parameters parameters = camera.getParameters(); //gets image parameters
            width = parameters.getPreviewSize().width; //determines image width
            height = parameters.getPreviewSize().height; //determines image height
            yuvImage = new YuvImage(data, ImageFormat.NV21, width, height, null); //exports image to yuv
            looped += 1; //logs iterations
        }
    };

    /**
     * converts the yuv image into a bitmap
     */
    private void convertImage() {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new Rect(0, 0, width, height), 0, out); //compresses yuv image into a jpeg
        byte[] imageBytes = out.toByteArray();
        image = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length); //turns the jpeg into a bitmap
    }

    /**
     * initializes the camera
     */
    public void startCam() {
        camera = ((FtcRobotControllerActivity) hardwareMap.appContext).camera;
        camera.setPreviewCallback(previewCallback); //sets the camera to the proper camera
        Camera.Parameters parameters = camera.getParameters(); //gets the camera's parameters
        data = parameters.flatten(); //flattens the parameters

        ((FtcRobotControllerActivity) hardwareMap.appContext).initPreview(camera, this, previewCallback);
    }

    /**
     * allows for the easy reading of a pixel by other functions
     * @param x the x coorinate of the pixel
     * @param y the y coordinate of the pixel
     * @return outputs the red, green, and blue vlues of the pixel
     */
    int[] pixel(int x, int y) {
        int pixelTarg = image.getPixel(x, y); //turns coordinates into one int
        int[] tmpArray = {red(pixelTarg), green(pixelTarg), blue(pixelTarg)}; //gets values from pixel
        return tmpArray; //exports rbg values
    }

    /**
     * exports the red value minus the blue value from the bottom left quadrant of the camera frame
     * @return total red-total blue
     */
    public int leftRed() {
        convertImage(); //converts the image
        int value = 0; //resets the value
        for (int x = 0; x < width / 2; x+= 3) { //reads the values of all of the pixels in the proper quadrant
            for (int y = height/2; y < height; y+=3) {
                value += pixel(x, y)[0]-pixel(x, y)[2];
            }
        }
        return value;
    }

    /**
     * exports the red value minus the blue value from the bottom right quadrant of the camera frame
     * @return total red minus total blue
     */
    public int rightRed() {
        convertImage(); //converts the image
        int value = 0; //resets the value
        for (int x = width - 1; x > width / 2; x -= 3) { //reads the values of all of the pixels in the proper quadrant
            for (int y = height/2; y < height; y = y + 3) {
                value += pixel(x, y)[0]-pixel(x, y)[2];
            }
        }
        return value;
    }
}
