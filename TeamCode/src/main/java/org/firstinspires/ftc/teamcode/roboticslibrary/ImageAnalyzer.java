package org.firstinspires.ftc.teamcode.roboticslibrary;

import android.content.ContextWrapper;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.util.Log;

import org.firstinspires.ftc.teamcode.RC;

import java.io.File;
import java.io.FileOutputStream;

/**
 * Created by Windows on 2016-01-24.
 */
public class ImageAnalyzer {
    private int [] image = null;
    private int width = 0;
    private int height = 0;
    private boolean analyzing = false;
    public static int red = Color.rgb(255, 0, 0);
    public static int blue = Color.rgb(0, 0, 255);
    long startTime = 0;

    public int numOfBlurredPixels = 50;

    public double blobAccuracy = 4;

    public void updateImage(Bitmap bit){
        if(!analyzing) {
            startTime = System.currentTimeMillis();
            width = bit.getWidth();
            height = bit.getHeight();
            image = new int[width * height];
            bit.getPixels(image, 0, width, 0, 0, width, height);
        }
    }

    private int getX(int index){
        return index % width;
    }

    private int getY(int index){
        return index / width;
    }

    public void applyColourFilter(){
        analyzing = true;
        for (int i = 0; i < image.length; i++) {
            image[i] = closeToColour(image[i]);
        }
        analyzing = false;
    }

    public String findBeaconConfig(int alliance){
        analyzing = true;
        double[] centres = findCenters();

        boolean redLeft = centres[0] > centres[1];
        analyzing = false;
        saveToInternalStorage(imageToBitmap(), "sample");
        if (redLeft && alliance == red) {
            return "Left";
        } else if (!redLeft && alliance == red) {
            return "Right";
        } else if (redLeft && alliance == blue) {
            return "Right";
        } else {
            return "Left";
        }
    }

    public double[] findCenters() {
        double redX = 0;
        double blueX = 0;
        int numBlue = 1;
        int numRed = 1;
        if (image != null) {
            for (int i = 0; i < image.length; i++) {
                if (image[i] == red) {
                    redX += getY(i);
                    numRed++;
                } else if (image[i] == blue) {
                    blueX += getY(i);
                    numBlue++;
                }
            }

            redX /= numRed;
            blueX /= numBlue;

            if (numRed < 9000) {
                redX = 0;
            }
            if (numBlue < 7000) {
                blueX = 0;
            }

            Log.i("Centres", "findCenters: Red: " + numRed + "," + redX + ", Blue: " + numBlue + "," + blueX);

        }

        return new double[]{redX, blueX};
    }

    public boolean isBusy(){
        return analyzing;
    }

    private long elapsedTime(){
        return System.currentTimeMillis() - startTime;
    }

    public Bitmap imageToBitmap(){
        return Bitmap.createBitmap(image, width, height, Bitmap.Config.ARGB_8888);
    }

    public static String saveToInternalStorage(Bitmap bitmapImage, String name) {
        ContextWrapper cw = new ContextWrapper(RC.c().getApplicationContext());
        // path to /data/data/yourapp/app_data/imageDir
        File directory = RC.c().getExternalFilesDir("");
        // Create imageDir
        File mypath = new File(directory, name + ".jpg");

        FileOutputStream fos = null;
        try {

            fos = new FileOutputStream(mypath);

            // Use the compress method on the BitMap object to write image to the OutputStream
            bitmapImage.compress(Bitmap.CompressFormat.PNG, 100, fos);

            fos.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return directory.getAbsolutePath();
    }

    public boolean hasImage(){
        return image != null;
    }

    public int closeToColour(int colour) {
        int blobAccuracy = 1;
        colour = Color.argb(255, Color.red(colour) - (int) (Color.red(colour) % (255 / blobAccuracy)),
                Color.green(colour) - (int) (Color.green(colour) % (255 / blobAccuracy)),
                Color.blue(colour) - (int) (Color.blue(colour) % (255 / blobAccuracy)));

        return colour;
    }

}
