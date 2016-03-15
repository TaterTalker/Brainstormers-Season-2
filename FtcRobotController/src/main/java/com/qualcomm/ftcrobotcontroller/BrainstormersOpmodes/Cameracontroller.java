package com.qualcomm.ftcrobotcontroller.BrainstormersOpmodes;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.ByteArrayOutputStream;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class Cameracontroller {

    LinearOpMode opMode;

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

    public Cameracontroller(LinearOpMode opMode){
        this.opMode = opMode;
    }


    /**
     * gets the red value of a getPixelColors
     * @param pixel the target getPixelColors
     * @return the getPixelColors's getRedInPixel value
     */
    private int getRedInPixel(int pixel) {
        return (pixel >> 16) & 0xff;
    }

    /**
     * gets the green value of a getPixelColors
     * @param pixel the target getPixelColors
     * @return the getPixelColors's getGreen value
     */
    private int getGreen(int pixel) {
        return (pixel >> 8) & 0xff;
    }

    /**
     * gets the blue value of a getPixelColors
     * @param pixel the target getPixelColors
     * @return the getPixelColors's getBlue value
     */
    private int getBlue(int pixel) {
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
        camera = ((FtcRobotControllerActivity) opMode.hardwareMap.appContext).camera;
        camera.setPreviewCallback(previewCallback); //sets the camera to the proper camera
        Camera.Parameters parameters = camera.getParameters(); //gets the camera's parameters
        data = parameters.flatten(); //flattens the parameters

        ((FtcRobotControllerActivity) opMode.hardwareMap.appContext).initPreview(camera, this, previewCallback);
    }

    /**
     * allows for the easy reading of a getPixelColors by other functions
     * @param x the x coorinate of the getPixelColors
     * @param y the y coordinate of the getPixelColors
     * @return outputs the getRedInPixel, getGreen, and getBlue vlues of the getPixelColors
     */
    int[] getPixelColors(int x, int y) {
        int pixelTarg = image.getPixel(x, y); //turns coordinates into one int
        int[] tmpArray = {getRedInPixel(pixelTarg), getGreen(pixelTarg), getBlue(pixelTarg)}; //gets values from getPixelColors
        return tmpArray; //exports rbg values
    }

    /**
     * exports the getRedInPixel value minus the getBlue value from the bottom left quadrant of the camera frame
     * @return total getRedInPixel-total getBlue
     */
    public int getRightRed() {
        convertImage(); //converts the image
        int value = 0; //resets the value
        for (int x = 0; x < width / 2; x+= 5) { //reads the values of all of the pixels in the proper quadrant
            for (int y = 0; y < height/2; y = y + 5) {
                value += getPixelColors(x, y)[0] - getPixelColors(x, y)[2];
            }
        }
        return value;
    }

    /**
     * exports the getRedInPixel value minus the getBlue value from the bottom right quadrant of the camera frame
     * @return total getRedInPixel minus total getBlue
     */
    public int getLeftRed() {
        convertImage(); //converts the image
        int value = 0; //resets the value
        for (int x = width - 1; x > width / 2; x -= 5) { //reads the values of all of the pixels in the proper quadrant
            for (int y = 0; y < height/2; y = y + 5) {
                value += getPixelColors(x, y)[0] - getPixelColors(x, y)[2];
            }
        }
        return value;
    }
}
