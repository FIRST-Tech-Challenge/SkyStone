package org.firstinspires.ftc.teamcode.helperclasses;

import android.graphics.Bitmap;

public class BitMapTesting {

    public static void main(String[] args){
        long startTime = System.nanoTime();
        Bitmap bitmapALPHA_8 = Bitmap.createBitmap(126, 126, Bitmap.Config.ALPHA_8);
        long endTime = System.nanoTime();

        System.out.println("Alpha 8 bitmap creation time: "+(endTime-startTime));


        startTime = System.nanoTime();
        Bitmap bitmapARGB_8888  = Bitmap.createBitmap(126, 126, Bitmap.Config.ARGB_8888 );
        endTime = System.nanoTime();

        System.out.println("ARGB 8888 bitmap creation time: "+(endTime-startTime));


        startTime = System.nanoTime();
        Bitmap bitmapRGB_565  = Bitmap.createBitmap(126, 126, Bitmap.Config.RGB_565);
        endTime = System.nanoTime();

        System.out.println("RGB 565 bitmap creation time: "+(endTime-startTime));

    }
}
