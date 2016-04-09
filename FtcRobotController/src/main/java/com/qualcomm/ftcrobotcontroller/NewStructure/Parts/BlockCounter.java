package com.qualcomm.ftcrobotcontroller.NewStructure.Parts;
import com.qualcomm.ftcrobotcontroller.NewStructure.AutoBot;


/**
 * Created by August on 4/9/2016.
 */
public class BlockCounter implements Runnable{

    AutoBot autoBot;

    public BlockCounter(AutoBot autoBot){
        this.autoBot = autoBot;
    }

    int[] getRowColors(int row){
        //try {
        autoBot.frontCam.convertImage();
        //} catch (Exception e){}

        int[] tmpArray = {0,0,0};
        int pixelX=1;
        while (pixelX<autoBot.frontCam.width) {
            tmpArray[0]+=autoBot.frontCam.getPixelColors(pixelX,row)[0];
            tmpArray[1]+=autoBot.frontCam.getPixelColors(pixelX,row)[1];
            tmpArray[2]+=autoBot.frontCam.getPixelColors(pixelX,row)[2];
            pixelX++;
        }
        return tmpArray;
    }

    @Override
    public void run() {
        int countInTarget=0;
        boolean isComplete=false;
        while(!isComplete){
            //autoBot.setCollectorDirection(1);
            int[] colors = getRowColors(120);
            if (colors[0]>18000){
                countInTarget++;
            }
            else if (countInTarget>0&&colors[2]<15000){
                countInTarget--;
            }

            if (countInTarget>20){
                isComplete=true;
            }

        }

        //autoBot.setCollectorDirection(-1);
    }
}
