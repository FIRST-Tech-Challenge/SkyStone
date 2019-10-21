package org.firstinspires.ftc.teamcode.roboticslibrary;

import android.content.ContextWrapper;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Log;

import org.firstinspires.ftc.teamcode.RC;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by Nirzvi on 2015-06-11.
 */
public class    ImageAnalysis {

    public double blobAccuracy = 3;

    public float blobEdgeAccuracy = 10;

    public int decRed = 50;

    public double edgeAccuracy = 10;
    public int rayIntensity = 50;
    public int numCentrePoints = 9;
    public int borderLimit = 1;
    public int switchLimit = 1;
    public int reflectLimit = 1;

    public int numOfBlobs = 100;

    int orange;

    public void increaseBlur () {

        switch (numOfBlobs) {

            case 1: numOfBlobs = 2; break;
            case 2: numOfBlobs = 4; break;
            case 4: numOfBlobs = 5; break;
            case 5: numOfBlobs = 10; break;
            case 10: numOfBlobs = 20; break;
            case 20: numOfBlobs = 25; break;
            case 25: numOfBlobs = 50; break;
            case 50: numOfBlobs = 100; break;
            case 100: numOfBlobs = 125; break;
            case 125: numOfBlobs = 250; break;
            case 250: numOfBlobs = 500; break;

        }

    }

    public void decreaseBlur () {

        switch (numOfBlobs) {

            case 2: numOfBlobs = 1; break;
            case 4: numOfBlobs = 2; break;
            case 5: numOfBlobs = 4; break;
            case 10: numOfBlobs = 5; break;
            case 20: numOfBlobs = 10; break;
            case 25: numOfBlobs = 20; break;
            case 50: numOfBlobs = 25; break;
            case 100: numOfBlobs = 50; break;
            case 125: numOfBlobs = 100; break;
            case 250: numOfBlobs = 125; break;
            case 500: numOfBlobs = 250; break;

        }

    }

    public Bitmap chrome = null;

    public void chromatize(int [] pixels, Bitmap bit){
        int width = bit.getWidth();
        int height = bit.getHeight();
        int [] colorTable = new int [width * height];
        int BLACK_THRESHOLD = 50;
        bit.getPixels(colorTable, 0, width, 0, 0, width, height);
        for(int i  = 0; i < pixels.length; i++){
            double red = Color.red(pixels[i]);
            double green = Color.green(pixels[i]);
            double blue = Color.blue(pixels[i]);
            if(red < BLACK_THRESHOLD && blue < BLACK_THRESHOLD && green < BLACK_THRESHOLD){
                pixels[i] = Color.BLACK;
                continue;
            }
            double cr = red / (red + green + blue);
            double cg = green / (red + green + blue);
            double cb = 1 - cr - cg;
            //Log.d("colors", "R: " + cr + " G: " + cg);
            if(cr > 0.7) cr = 0.7;
            if(cg > 0.85) cg = 0.85;

            int x = (int)(cr * (width - 1) / 0.7);
            int y = (int)( cg * (height - 1) / 0.85);
            //Log.d("index", "x " + x  + " y " + y);
            pixels[i] = colorTable[x + y * width];
        }
    }

    public String saveToInternalStorage(Bitmap bitmapImage, String name){
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

    public Bitmap chrome(Bitmap img){
        int[] pixels = new int [img.getWidth() * img.getHeight()];
        img.getPixels(pixels, 0, img.getWidth(), 0, 0, img.getWidth(), img.getHeight());
        chromatize(pixels, chrome);
        RC.t.addData("Right", getSideOfColour(pixels, Color.parseColor("#FE0000")));
        return Bitmap.createBitmap(pixels, img.getWidth(), img.getHeight(), Bitmap.Config.ARGB_8888);
    }

    public String getSideOfColour(int [] image, int toFind){
        int red = 0;
        int blue = 0;
        for (int i = 0; i < image.length; i++) {
            if(image[i] == toFind){

            }
        }
        return "";
    }
    public int averageColour(int[] pixels) {
        int r = 0;
        int g = 0;
        int b = 0;

        for (int i = 0; i < pixels.length; i++) {
            r += Color.red(pixels[i]);
            g += Color.green(pixels[i]);
            b += Color.blue(pixels[i]);
        }

        r /= pixels.length;
        g /= pixels.length;
        b /= pixels.length;

        return Color.rgb(r, g, b);
    }

    public Bitmap blurring (Bitmap bit) {

        int imageWidth = bit.getWidth();
        int imageHeight = bit.getHeight();
        int[] pixels = new int[numOfBlobs * numOfBlobs];
        int centreCount = 0;
        int[] tempPixels = new int[imageHeight * imageWidth];
        int[] pixCoords = new int[pixels.length];

        bit.getPixels(tempPixels, 0, imageWidth, 0, 0, imageWidth, imageHeight);

        for (int i = 0; i < numOfBlobs; i++) {
            for (int j = 0; j < numOfBlobs; j++) {
                pixCoords[centreCount] = (i * imageWidth / numOfBlobs) + (j * imageWidth * imageHeight / numOfBlobs);

                int[] avgPix = new int [(imageWidth / numOfBlobs) * (imageHeight / numOfBlobs)];
                int avgCount = 0;

                for (int k = 0; k < imageWidth / numOfBlobs; k++) {
                    for (int l = 0; l < imageHeight / numOfBlobs; l++) {
                        if (pixCoords[centreCount] + k < imageWidth)
                            avgPix[avgCount] = tempPixels[pixCoords[centreCount] + k + (j * imageWidth)];
                    }
                }

//                tempPixels[centreCount] = averageColour(avgPix);

                pixels[centreCount] = closeToColour(tempPixels[pixCoords[centreCount]]);
                centreCount++;
            }
        }

        for (int i = 0; i < pixels.length; i++) {
            tempPixels[pixCoords[i]] = pixels[i];
        }

        tempPixels = expandPixels(tempPixels, imageWidth, imageHeight);

        bit = Bitmap.createBitmap(tempPixels, imageWidth, imageHeight, Bitmap.Config.ARGB_8888);

        return bit;
    }

    public int borderColour (int[] pixels, int imageHeight) {
        int borderCount = 0;

        for (int i = 0; i < pixels.length; i++) {
            if (pixels[i] == Color.BLACK) {
                borderCount++;
            }
        }

        if (borderCount > (imageHeight / numOfBlobs) - 2)
            return Color.BLACK;

        return Color.WHITE;

    }

    public Bitmap compilation (Bitmap bit) {

        return blurMethod(getAmbientEdges(bit), false);

//        return findBlobWithInts(blurMethodInts(getAmbientEdges(bit), false), bit.getWidth());
    }

    public int[] blurMethodInts (Bitmap bit, boolean colour) {

        int imageWidth = bit.getWidth();
        int imageHeight = bit.getHeight();
        int[] tempPixels = new int[imageHeight * imageWidth];
        int[] pixels = new int[numOfBlobs * numOfBlobs];
        int[] pixCoords = new int[numOfBlobs * numOfBlobs];
        int[] blobIndex = new int[tempPixels.length];
        int[] storePix = new int[pixels.length];
        List<Integer> blobs = new ArrayList<Integer>();
        int colourCount = 0;
        int centreCount = 0;

        bit.getPixels(tempPixels, 0, imageWidth, 0, 0, imageWidth, imageHeight);

        for (int i = 0; i < numOfBlobs; i++) {
            for (int j = 0; j < numOfBlobs; j++) {
                pixCoords[centreCount] = (i * imageWidth / numOfBlobs) + (j * imageWidth * imageHeight / numOfBlobs);

                int[] avgPix = new int [(imageWidth / numOfBlobs) * (imageHeight / numOfBlobs)];
                int avgCount = 0;

                for (int k = 0; k < imageWidth / numOfBlobs; k++) {
                    for (int l = 0; l < imageHeight / numOfBlobs; l++) {
                        if (pixCoords[centreCount] + k < imageWidth)
                            avgPix[avgCount] = tempPixels[pixCoords[centreCount] + k + (j * imageWidth)];
                    }
                }

//                tempPixels[centreCount] = averageColour(avgPix);
                tempPixels[centreCount] = borderColour(avgPix, imageHeight);

                pixels[centreCount] = closeToColour(tempPixels[pixCoords[centreCount]]);
                centreCount++;
            }
        }

        return pixels;
    }

    public int[] getBlurryEdges (int[] pixels, int imageWidth) {

        int[] storePixels = pixels.clone();

        for (int i = 0; i < pixels.length; i++) {
            if (i % imageWidth > 0 && compareColours(storePixels[i], storePixels[i - 1])) {
                pixels[i] = Color.BLACK;
            } else {
                pixels[i] = Color.WHITE;
            }
        }

        for (int i = 0; i < pixels.length; i += imageWidth) {
            if (i / imageWidth > 0 && compareColours(storePixels[i], storePixels[i - imageWidth])) {
                pixels[i] = Color.BLACK;
            } else {
                pixels[i] = Color.WHITE;
            }
        }

        return pixels;
    }

    public Bitmap blurMethod (Bitmap bit, boolean colour) {

        int imageWidth = bit.getWidth();
        int imageHeight = bit.getHeight();
        int[] tempPixels = new int[imageHeight * imageWidth];
        int[] pixels = new int[numOfBlobs * numOfBlobs];
        int[] pixCoords = new int[numOfBlobs * numOfBlobs];
        int[] blobIndex = new int[tempPixels.length];
        int[] storePix = new int[pixels.length];
        List<Integer> blobs = new ArrayList<Integer>();
        int colourCount = 0;
        int centreCount = 0;

        bit.getPixels(tempPixels, 0, imageWidth, 0, 0, imageWidth, imageHeight);

        for (int i = 0; i < numOfBlobs; i++) {
            for (int j = 0; j < numOfBlobs; j++) {
                pixCoords[centreCount] = (i * imageWidth / numOfBlobs) + (j * imageWidth * imageHeight / numOfBlobs);

                int[] avgPix = new int [(imageWidth / numOfBlobs) * (imageHeight / numOfBlobs)];
                int avgCount = 0;

                for (int k = 0; k < imageWidth / numOfBlobs; k++) {
                    for (int l = 0; l < imageHeight / numOfBlobs; l++) {
                        if (pixCoords[centreCount] + k < imageWidth)
                            avgPix[avgCount] = tempPixels[pixCoords[centreCount] + k + (j * imageWidth)];
                    }
                }

//                tempPixels[pixCoords[centreCount]] = averageColour(avgPix);
//                tempPixels[centreCount] = borderColour(avgPix, imageHeight);

                pixels[centreCount] = tempPixels[pixCoords[centreCount]];
//                pixels[centreCount] = closeToColour(tempPixels[pixCoords[centreCount]]);
                centreCount++;
            }
        }

        pixels = getBlurryEdges(pixels, numOfBlobs);

        for (int i = 0; i < pixels.length; i++) {
            storePix[i] = pixels[i];
        }

        if (colour) {
            for (int i = 0; i < pixels.length; i++) {
                if (i % numOfBlobs > 0 && !compareColours(storePix[i], storePix[i - 1])) {
                    pixels[i] = pixels[i - 1];
                } else if (i / numOfBlobs > 0 && !compareColours(storePix[i], storePix[i - numOfBlobs])) {
                    pixels[i] = pixels[i - numOfBlobs];
                } else {
                    pixels[i] = colourCount * -4500 - 1;
                    colourCount++;
                }
            }

            for (int i = 0; i < pixels.length; i++) {
                if (i % numOfBlobs > 0 && !compareColours(storePix[i], storePix[i - 1])
                        && pixels[i] != pixels[i - 1]) {
                    replace (pixels, pixels[i], pixels[i - 1]);
                } else if (i / numOfBlobs > 0 && !compareColours(storePix[i], storePix[i - numOfBlobs])
                        && pixels[i] != pixels[i - numOfBlobs]) {
                    replace(pixels, pixels[i], pixels[i - numOfBlobs]);
                }
            }
        }

        for (int i = 0; i < pixels.length; i++) {
            tempPixels[pixCoords[i]] = pixels[i];
        }

        tempPixels = expandPixels(tempPixels, imageWidth, imageHeight);

        bit = Bitmap.createBitmap(tempPixels, imageWidth, imageHeight, Bitmap.Config.ARGB_8888);

        return bit;
    }

    public boolean compareColours (int compare1, int compare2) {

        return (Math.abs(Color.red(compare1) - Color.red(compare2)) > blobEdgeAccuracy
                || Math.abs(Color.green(compare1) - Color.green(compare2)) > blobEdgeAccuracy
                || Math.abs(Color.blue(compare1) - Color.blue(compare2)) > blobEdgeAccuracy);

//        return compare1 != compare2;
    }

    public int[] expandPixels (int[] srcPix, int imageWidth, int imageHeight) {

        int[] tempPixels = new int[imageWidth * imageHeight];

        for (int i = 0; i < tempPixels.length; i++) {

            int coord = i;
            coord -= i % (imageWidth / numOfBlobs);
            coord -= ((i / imageWidth) % (imageHeight / numOfBlobs)) * imageWidth;

            tempPixels[i] = srcPix[coord];
        }

        return tempPixels;
    }

    public Bitmap temporary (Bitmap bit) {

        int[] pixels = new int[bit.getWidth() * bit.getHeight()];

        bit.getPixels(pixels, 0, bit.getWidth(), 0, 0, bit.getWidth(), bit.getHeight());

        for (int i = 0; i < pixels.length; i++) {
            pixels[i] = closeToColour(pixels[i]);
        }

        bit = Bitmap.createBitmap(pixels, bit.getWidth(), bit.getHeight(), Bitmap.Config.ARGB_8888);

        return bit;
    }

    public int closeToColour (int colour) {

        int r = Color.red(colour);
        int g = Color.green(colour);
        int b = Color.blue(colour);

//        if (r > b && r > g) {
//            b = 0;
//            g = 0;
//            r = 255;
//        } else if (b > r && b > g) {
//            r = 0;
//            g = 0;
//            b = 255;
//        } else if (g > r && g > b) {
//            r = 0;
//            b = 0;
//            g = 255;
//        } else {
//            r = 0;
//            b = 0;
//            g = 0;
//        }

        r -= decRed;

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

//        hsv[2] = 1;
//        hsv[1] = 1;

        hsv[2] = Math.round(hsv[2] * 2) / 2f;
        hsv[1] = Math.round(hsv[1] * 2) / 2f;
//
//        hsv[2] = Math.round(hsv[2] + 0.4f);
//        hsv[1] = Math.round(hsv[1] + 0.4f);

        hsv[0] = closestColour((int) hsv[0]);
//        hsv[0] -= 100;

        int newColour = Color.HSVToColor(255, hsv);

        r = Color.red(newColour);
        g = Color.green(newColour);
        b = Color.blue(newColour);


        newColour = Color.rgb(r, g, b);

        newColour = Color.argb(255, (int) (Color.red(newColour) - (Color.red(newColour) % 255 / blobAccuracy)),
                (int) (Color.green(newColour) - (Color.green(newColour) % 255 / blobAccuracy)),
                (int) (Color.blue(newColour) - (Color.blue(newColour) % 255 / blobAccuracy)));

        int difference = Math.abs(newColour - chooseColour(0));
        int newNewColour = chooseColour(0);

        for (int i = 1; i < 6; i++) {
            if (difference < Math.abs(newColour - chooseColour(i))) {
                newNewColour = chooseColour(i);
            }
        }

        return Color.argb(255, r, g, b);
    }

    public int compareClosestColours (int colour1, int colour2) {

        return Math.abs(Color.red(colour1) - Color.red(colour2))
                + Math.abs(Color.green(colour1) - Color.green(colour2))
                + Math.abs(Color.blue(colour1) - Color.blue(colour2));

    }

    public int chooseColour (int colourCount) {
        switch (colourCount) {
            case 0: return Color.argb(255, 255, 0, 0);
            case 1: return Color.argb(255, 0, 255, 0);
            case 2: return Color.argb(255, 0, 0, 255);
            case 3: return Color.argb(255, 255, 239, 0);
            case 4: return Color.argb(255, 213, 0, 255);
            case 5: return Color.argb(255, 255, 255, 255);
            case 6: return Color.argb(255, 0, 0, 0);
            default: return Color.argb(255, 0, 255, 255);
        }
    }

    public int closestColour (int colour) {

        return (Math.round(colour / (360f / blobEdgeAccuracy)) * 60);

//        int difference;
//        int closestColour;
//
//
//        difference = compareClosestColours(colour, chooseColour(0));
//        closestColour = chooseColour(0);
//
//        for (int i = 1; i < 8; i++) {
//            if (difference < compareClosestColours(colour, chooseColour(i))) {
//                difference = compareClosestColours(colour, chooseColour(i));
//                closestColour = chooseColour(i);
//            }
//        }
//
//        return closestColour;
    }

    public Bitmap classic (Bitmap bit) {

        int imageWidth = bit.getWidth();
        int imageHeight = bit.getHeight();
        int[] pixels = new int[imageWidth * imageHeight];
        List<String> blobs = new ArrayList<String>();
        int colourCounter = 0;
        boolean newColour;
        boolean newBlob;

        bit.getPixels(pixels, 0, imageWidth, 0, 0, imageWidth, imageHeight);

        for (int i = 0; i < pixels.length; i++) {

            pixels[i] = closeToColour(pixels[i]);
//            Log.i("PixColour", "" + pixels[i]);

            newBlob = true;
            for (int j = 0; j < blobs.size(); j++) {

                if (getColour(blobs.get(j)) == pixels[i]) {
                    if (nearBlob(blobs.get(j), i, imageWidth)) {
                        newBlob = false;
                        blobs.set(j, blobs.get(j) + i + "|");
                        break;
                    }
                }

            }

            if (newBlob) {
                colourCounter++;
                blobs.add("" + pixels[i] + "|" + (colourCounter * -4500 - 1) + "?" + i + "|");
            }

        }

        return bit;
    }

    public boolean nearBlob (String toCheck, int coord, int imageWidth) {

        int index = toCheck.indexOf('?');
        String toUse = toCheck;

        toUse = toUse.substring(index + 1);

//        Log.i("toUse", toUse);

        try {
            while (true) {
                int blobCoord;

//                Log.i("BLOBCOORD", toUse.substring(0, toUse.indexOf("|")));

                blobCoord = Integer.parseInt(toUse.substring(0, toUse.indexOf("|")));

                if (Math.abs(getX(blobCoord, imageWidth) - getX(coord, imageWidth)) < 10
                        && Math.abs(getY(blobCoord, imageWidth) - getY(coord, imageWidth)) < 10) {
                    return true;
                }

                if (toCheck.indexOf("|", index) == toCheck.lastIndexOf("|"))
                    break;

                toUse = toUse.substring(toUse.indexOf("|") + 1);

//                if (toUse != null)
//                    Log.i("toUse SECOND", toUse);

//                Log.i("CHECK", "GOOD");
            }
//        } catch (StringIndexOutOfBoundsException e) {
//            Log.i("toUse from nearBlob()", toCheck);
//            Log.i("INDEX OUT OF BOUNDS", "!!!!!!!!" + e.getCause());
//        } catch (NumberFormatException e) {
//            Log.i("toUse from nearBlob()", toCheck);
//            Log.i("INTEGER NOT PROPER", "??????");
        } catch (Exception e) {
//            Log.i("toUse from nearBlob()", toCheck);
//            Log.i("UNKNOWN", "}}}}}}}");
        }

        return false;
    }

    public int getX (int coord, int imageWidth) {
        return coord % imageWidth;
    }
    public int getY (int coord, int imageWidth) {
        return coord / imageWidth;
    }

    public int getColour (String colour) {

        int colourInt = 0;
        int index = 0;

        try {

            for (int i = 0; colour.charAt(i) != '|'; i++)
                index++;

            colourInt = Integer.parseInt(colour.substring(0, index));
//            Log.i("Colour", "" + colourInt);

        } catch (Exception e) {
//            Log.i("Colour from getColour()", colour);
        }

        return colourInt;
    }

    public Bitmap getEdges (Bitmap bit) {

        int bitWidth = bit.getWidth();
        int bitHeight = bit.getHeight();
        int[] pixels = new int[bit.getWidth() * bit.getHeight()];
        int[] storePixels = new int[pixels.length];
        Bitmap newBit;
        int difference = Math.abs(bit.getWidth() - bit.getHeight());
        bitWidth -= difference;

        newBit = bit.createBitmap(bit.getWidth(), bit.getHeight(), Bitmap.Config.ARGB_8888);

        int x;
        int y;
        int newCoord;

        Canvas can = new Canvas(newBit);
        Paint colourPaint = new Paint();

        colourPaint.setColor(Color.WHITE);
        can.drawRect(0, 0, newBit.getWidth(), newBit.getHeight(), colourPaint);
        colourPaint.setColor(Color.BLACK);

        bit.getPixels(pixels, 0, bit.getWidth(), 0, 0, bit.getWidth(), bit.getHeight());

        for (int i = 1; i < pixels.length; i++) {
            if (closeToColour(pixels[i]) != closeToColour(pixels[i - 1])) {
                x = i % bit.getWidth();
                y = i / bit.getWidth();
                can.drawRect(x, y, x + 1, y + 1, colourPaint);
            }
        }

        for (int i = 0; i < pixels.length; i++) {
            storePixels[i] = pixels[i];
        }

        for (int i = 1; i < pixels.length; i++) {

            if (!((i % bitWidth) >= bitHeight))
                newCoord = (i % bitWidth * bit.getWidth()) + (i / bitWidth);
            else
                newCoord = 0;

            pixels[i] = storePixels[newCoord];
        }

        for (int i = 1; i < pixels.length; i++) {
            if (closeToColour(pixels[i]) != closeToColour(pixels[i - 1])) {
                y = i % bitWidth;
                x = i / bitWidth;
                can.drawLine(x, y, x + 1, y + 1, colourPaint);
            }
        }

        return newBit;

    }


    public Bitmap getAmbientEdges (Bitmap bit) {

        int bitWidth = bit.getWidth();
        int bitHeight = bit.getHeight();
        int[] pixels = new int[bit.getWidth() * bit.getHeight()];
        int[] storePixels = new int[pixels.length];
        Bitmap newBit;
        int difference = Math.abs(bit.getWidth() - bit.getHeight());
        bitWidth -= difference;

        newBit = bit.createBitmap(bit.getWidth(), bit.getHeight(), Bitmap.Config.ARGB_8888);

        int x;
        int y;
        int newCoord;

        Canvas can = new Canvas(newBit);
        Paint colourPaint = new Paint();

        colourPaint.setColor(Color.WHITE);
        can.drawRect(0, 0, newBit.getWidth(), newBit.getHeight(), colourPaint);
        colourPaint.setColor(Color.BLACK);

        bit.getPixels(pixels, 0, bit.getWidth(), 0, 0, bit.getWidth(), bit.getHeight());

        for (int i = 1; i < pixels.length; i++) {
            if ((i % bit.getWidth() > 0 && (Math.abs(Color.red(pixels[i - 1]) - Color.red(pixels[i])) > edgeAccuracy
                || Math.abs(Color.green(pixels[i - 1]) - Color.green(pixels[i])) > edgeAccuracy
                || Math.abs(Color.blue(pixels[i - 1]) - Color.blue(pixels[i])) > edgeAccuracy))) {
                x = i % bit.getWidth();
                y = i / bit.getWidth();
                can.drawRect(x, y, x + 1, y + 1, colourPaint);
            } else if (i % bit.getWidth() == 0 || i % bit.getWidth() == bit.getWidth() - 1
                    || i / bit.getWidth() == 0 || i / bit.getWidth() == bit.getHeight() - 1) {
                x = i % bit.getWidth();
                y = i / bit.getWidth();
                can.drawRect(x, y, x + 1, y + 1, colourPaint);
            }
        }

        for (int i = 0; i < pixels.length; i++) {
            storePixels[i] = pixels[i];
        }

        for (int i = 1; i < pixels.length; i++) {

            if (!((i % bitWidth) >= bitHeight))
                newCoord = (i % bitWidth * bit.getWidth()) + (i / bitWidth);
            else
                newCoord = 0;

            pixels[i] = storePixels[newCoord];
        }

        for (int i = 1; i < pixels.length; i++) {
            if ((i % bit.getWidth() > 0 && (Math.abs(Color.red(pixels[i - 1]) - Color.red(pixels[i])) > edgeAccuracy
                    || Math.abs(Color.green(pixels[i - 1]) - Color.green(pixels[i])) > edgeAccuracy
                    || Math.abs(Color.blue(pixels[i - 1]) - Color.blue(pixels[i])) > edgeAccuracy))) {
                y = i % bitWidth;
                x = i / bitWidth;
                can.drawLine(x, y, x + 1, y + 1, colourPaint);
            }
        }

//        int goodPixel;
//        int areaOfCheck = 5;
//        int bordLimitReq = 3;
//
//        for (int i = 0; i < pixels.length; i++) {
//
//            if (pixels[i] == Color.BLACK) {
//                goodPixel = 0;
//                for (int j = 0; j < areaOfCheck; j++) {
//                    for (int k = 0; k < areaOfCheck; k++) {
//                        if (i - areaOfCheck / 2 + j - ((k - areaOfCheck / 2) * bit.getWidth()) >= 0 && i - areaOfCheck / 2 + j - ((k - areaOfCheck / 2) * bit.getWidth()) < pixels.length - 1 && i - areaOfCheck / 2 + j - ((k - areaOfCheck / 2) * bit.getWidth()) != i && pixels[i - areaOfCheck / 2 + j - ((k - areaOfCheck / 2) * bit.getWidth())] == Color.BLACK) {
//                            goodPixel++;
//
//                            if (goodPixel >= bordLimitReq)
//                                break;
//                        }
//                    }
//                }
//
//                if (goodPixel < bordLimitReq)
//                    pixels[i] = Color.WHITE;
//            }
//
//        }

        return newBit;

    }

    public int[] findBlobLabels (Bitmap img){
        int width = img.getWidth();
        int height = img.getHeight();
        int check = 1;
        int x = 0;
        int y = 0;
        int [] pixels = new int[width * height];
        int [] label = new int [width * height];

        img = getAmbientEdges(img);

        img.getPixels(pixels, 0, width, 0, 0, width, height);

        Arrays.fill(label, -1);

        label[0] = 0;

        for(int i = 0; i < pixels.length - width - 1; i++){
            int same = -1;
            if(i % width == width - 1){
                i++;
                continue;
            }

            //checkTimer north
            if(pixels[i] == pixels[i + width]){
                label[i + width] = label[i];
                same = 2;
            }
            //checkTimer west
            if(pixels[i] == pixels[i + 1]){
                label[i + 1] = label[i];
                same = 1;
            }

            //checkTimer northwest
            if(pixels[i] == pixels[i + width + 1] && same != -1){
                label[i + width + 1] = label[i];
            }

            if(pixels[i] == Color.BLACK) {
                if (pixels[i + 1] == pixels[i + width] && pixels[i + 1] == pixels[i + width + 1]) {
                    if (label[i + width] != -1) {
                        label[i + 1] = label[i + width];
                        label[i + width + 1] = label[i + width];
                    } else if (label[i + 1] != -1) {
                        label[i + width] = label[i + 1];
                        label[i + width + 1] = label[i + 1];
                    } else {
                        label[i + 1] = check;
                        label[i + width] = check;
                        label[i + width + 1] = check;
                        check++;
                    }
                } else if (pixels[i + 1] == pixels[i + width]) {
                    if (label[i + 1] != -1) {
                        label[i + width] = label[i + 1];
                    } else if (label[i + width] != -1) {
                        label[i + 1] = label[i + width];
                    } else {
                        label[i + width] = check;
                        label[i + 1] = check;
                        check++;
                    }
                } else if (pixels[i + 1] == pixels[i + width + 1]) {
                    if (label[i + 1] != -1) {
                        label[i + width + 1] = label[i + 1];
                    } else {
                        label[i + 1] = check;
                        label[i + width + 1] = check;
                        check++;
                    }
                } else if (pixels[i + width] == pixels[i + width + 1]) {
                    if (label[i + width] != -1) {
                        label[i + width + 1] = label[i + width];
                    } else {
                        label[i + width] = check;
                        label[i + width + 1] = check;
                        check++;
                    }
                }
            }
        }

        for(int i = pixels.length - 1; i >= width + 1; i--) {
            if(label[i] != label[i - 1] && pixels[i] == pixels[i - 1]){
                replace(label, label[i - 1], label[i]);
            }
            if(label[i] != label[i - width] && pixels[i] == pixels[i - width]){
                replace(label, label[i - width], label[i]);
            }
        }

        return label;
    }


    public Bitmap findBlobs(Bitmap img){
        int width = img.getWidth();
        int height = img.getHeight();
        int check = 1;
        int x = 0;
        int y = 0;
        int [] pixels = new int [width * height];
        int [] label = new int [width * height];
        List<String> labels = new ArrayList<String>();

        img.getPixels(pixels, 0, width, 0, 0, width, height);

        Arrays.fill(label, -1);

        label[0] = 0;

        for(int i = 0; i < pixels.length - width - 1; i++){
            int same = -1;
            if(i % width == width - 1){
                i++;
                continue;
            }

            //checkTimer north
            if(pixels[i] == pixels[i + width]){
                label[i + width] = label[i];
                same = 2;
            }
            //checkTimer west
            if(pixels[i] == pixels[i + 1]){
                label[i + 1] = label[i];
                same = 1;
            }

            //checkTimer northwest
            if(pixels[i] == pixels[i + width + 1] && same != -1){
                label[i + width + 1] = label[i];
            }

            if(pixels[i] == Color.BLACK) {
                if (pixels[i + 1] == pixels[i + width] && pixels[i + 1] == pixels[i + width + 1]) {
                    if (label[i + width] != -1) {
                        label[i + 1] = label[i + width];
                        label[i + width + 1] = label[i + width];
                    } else if (label[i + 1] != -1) {
                        label[i + width] = label[i + 1];
                        label[i + width + 1] = label[i + 1];
                    } else {
                        label[i + 1] = check;
                        label[i + width] = check;
                        label[i + width + 1] = check;
                        check++;
                    }
                } else if (pixels[i + 1] == pixels[i + width]) {
                    if (label[i + 1] != -1) {
                        label[i + width] = label[i + 1];
                    } else if (label[i + width] != -1) {
                        label[i + 1] = label[i + width];
                    } else {
                        label[i + width] = check;
                        label[i + 1] = check;
                        check++;
                    }
                } else if (pixels[i + 1] == pixels[i + width + 1]) {
                    if (label[i + 1] != -1) {
                        label[i + width + 1] = label[i + 1];
                    } else {
                        label[i + 1] = check;
                        label[i + width + 1] = check;
                        check++;
                    }
                } else if (pixels[i + width] == pixels[i + width + 1]) {
                    if (label[i + width] != -1) {
                        label[i + width + 1] = label[i + width];
                    } else {
                        label[i + width] = check;
                        label[i + width + 1] = check;
                        check++;
                    }
                }
            }
        }

        for(int i = pixels.length - 1; i >= width + 1; i--) {
            if(label[i] != label[i - 1] && pixels[i] == pixels[i - 1]){
                replace(label, label[i - 1], label[i]);
            }
            if(label[i] != label[i - width] && pixels[i] == pixels[i - width]){
                replace(label, label[i - width], label[i]);
            }
        }

        Bitmap bitsy =  Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        Paint color = new Paint();
        Canvas pic = new Canvas(bitsy);
        for(int i = 0; i < pixels.length; i++) {
            x = i % width;
            y = i / width;
            color.setColor(label[i] * -4500 - 1);
            pic.drawRect(x, y, x + 2, y + 2, color);
        }//for


        bitsy =  Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        pic = new Canvas(bitsy);
        for(int i = 0; i < pixels.length; i++) {
            x = i % width;
            y = i / width;
            color.setColor(label[i] * -4500 - 1);
            pic.drawRect(x, y, x + 2, y + 2, color);
        }//for

        //shapeProcessor(label, width);

        return bitsy;
    }

    public void replace(int[] label, int l1, int l2) {

        for(int i = 0; i < label.length; i++){
            if(label[i] == l1) {
                label[i] = l2;
            }
        }
    }

    public Bitmap findBlobWithInts (int[] pixels, int imageWidth){
        int width = imageWidth;
        int height = pixels.length / imageWidth;
        int check = 1;
        int x = 0;
        int y = 0;
        int [] label = new int [pixels.length];
        List<String> labels = new ArrayList<String>();

        Arrays.fill(label, -1);

        label[0] = 0;

        for(int i = 0; i < pixels.length - width - 1; i++){
            int same = -1;
            if(i % width == width - 1){
                i++;
                continue;
            }

            //checkTimer north
            if(pixels[i] == pixels[i + width]){
                label[i + width] = label[i];
                same = 2;
            }
            //checkTimer west
            if(pixels[i] == pixels[i + 1]){
                label[i + 1] = label[i];
                same = 1;
            }

            //checkTimer northwest
            if(pixels[i] == pixels[i + width + 1] && same != -1){
                label[i + width + 1] = label[i];
            }

            if(pixels[i] == Color.BLACK) {
                if (pixels[i + 1] == pixels[i + width] && pixels[i + 1] == pixels[i + width + 1]) {
                    if (label[i + width] != -1) {
                        label[i + 1] = label[i + width];
                        label[i + width + 1] = label[i + width];
                    } else if (label[i + 1] != -1) {
                        label[i + width] = label[i + 1];
                        label[i + width + 1] = label[i + 1];
                    } else {
                        label[i + 1] = check;
                        label[i + width] = check;
                        label[i + width + 1] = check;
                        check++;
                    }
                } else if (pixels[i + 1] == pixels[i + width]) {
                    if (label[i + 1] != -1) {
                        label[i + width] = label[i + 1];
                    } else if (label[i + width] != -1) {
                        label[i + 1] = label[i + width];
                    } else {
                        label[i + width] = check;
                        label[i + 1] = check;
                        check++;
                    }
                } else if (pixels[i + 1] == pixels[i + width + 1]) {
                    if (label[i + 1] != -1) {
                        label[i + width + 1] = label[i + 1];
                    } else {
                        label[i + 1] = check;
                        label[i + width + 1] = check;
                        check++;
                    }
                } else if (pixels[i + width] == pixels[i + width + 1]) {
                    if (label[i + width] != -1) {
                        label[i + width + 1] = label[i + width];
                    } else {
                        label[i + width] = check;
                        label[i + width + 1] = check;
                        check++;
                    }
                }
            }
        }

        for(int i = pixels.length - 1; i >= width + 1; i--) {
            if(label[i] != label[i - 1] && pixels[i] == pixels[i - 1]){
                replace(label, label[i - 1], label[i]);
            }
            if(label[i] != label[i - width] && pixels[i] == pixels[i - width]){
                replace(label, label[i - width], label[i]);
            }
        }

        Bitmap bitsy =  Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        Paint color = new Paint();
        Canvas pic = new Canvas(bitsy);
        for(int i = 0; i < pixels.length; i++) {
            x = i % width;
            y = i / width;
            color.setColor(label[i] * -4500 - 1);
            pic.drawRect(x, y, x + 2, y + 2, color);
        }//for


        bitsy =  Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        pic = new Canvas(bitsy);
        for(int i = 0; i < pixels.length; i++) {
            x = i % width;
            y = i / width;
            color.setColor(label[i] * -4500 - 1);
            pic.drawRect(x, y, x + 2, y + 2, color);
        }//for

        //shapeProcessor(label, width);

        return bitsy;
    }

    public void replace(List<Integer> label, int l1, int l2) {

        while (true) {
            if(label.contains(l1)) {
                label.set(label.indexOf(l1), l2);
            } else {
                break;
            }
        }
    }




















    public Bitmap speedBlobs (Bitmap img) {

        int width = img.getWidth();
        int height = img.getHeight();
        int check = 1;
        int x = 0;
        int y = 0;
        int [] pixels = new int[width * height];
        int [] label = new int [width * height];
        List<Integer> newLabels = new ArrayList<Integer>();

        img = getAmbientEdges(img);

        img.getPixels(pixels, 0, width, 0, 0, width, height);

        Arrays.fill(label, -1);

        label[0] = 0;

        for(int i = 0; i < pixels.length - width - 1; i++){
            int same = -1;
            if(i % width == width - 1){
                i++;
                continue;
            }

            //checkTimer north
            if(pixels[i] == pixels[i + width]){
                label[i + width] = label[i];
                same = 2;
            }
            //checkTimer west
            if(pixels[i] == pixels[i + 1]){
                label[i + 1] = label[i];
                same = 1;
            }

            //checkTimer northwest
            if(pixels[i] == pixels[i + width + 1] && same != -1){
                label[i + width + 1] = label[i];
            }

            if(pixels[i] == Color.BLACK) {
                if (pixels[i + 1] == pixels[i + width] && pixels[i + 1] == pixels[i + width + 1]) {
                    if (label[i + width] != -1) {
                        label[i + 1] = label[i + width];
                        label[i + width + 1] = label[i + width];
                    } else if (label[i + 1] != -1) {
                        label[i + width] = label[i + 1];
                        label[i + width + 1] = label[i + 1];
                    } else {
                        label[i + 1] = check;
                        label[i + width] = check;
                        label[i + width + 1] = check;
                        check++;
                    }
                } else if (pixels[i + 1] == pixels[i + width]) {
                    if (label[i + 1] != -1) {
                        label[i + width] = label[i + 1];
                    } else if (label[i + width] != -1) {
                        label[i + 1] = label[i + width];
                    } else {
                        label[i + width] = check;
                        label[i + 1] = check;
                        check++;
                    }
                } else if (pixels[i + 1] == pixels[i + width + 1]) {
                    if (label[i + 1] != -1) {
                        label[i + width + 1] = label[i + 1];
                    } else {
                        label[i + 1] = check;
                        label[i + width + 1] = check;
                        check++;
                    }
                } else if (pixels[i + width] == pixels[i + width + 1]) {
                    if (label[i + width] != -1) {
                        label[i + width + 1] = label[i + width];
                    } else {
                        label[i + width] = check;
                        label[i + width + 1] = check;
                        check++;
                    }
                }
            }
        }

        for (int i = 0; i < label.length; i++) {
            newLabels.add(label[i]);
        }

        for(int i = pixels.length - 1; i >= width + 1; i--) {
            if(label[i] != label[i - 1] && pixels[i - 1] != Color.BLACK){
                replace(newLabels, label[i - 1], label[i]);
            }
            if(label[i] != label[i - width] && pixels[i - width] != Color.BLACK){
                replace(newLabels, label[i - width], label[i]);
            }
        }

        for (int i = 0; i < label.length; i++) {
            label[i] = newLabels.get(i);
        }

        Bitmap bitsy =  Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        Paint color = new Paint();
        Canvas pic = new Canvas(bitsy);
        for(int i = 0; i < pixels.length; i++) {
            x = i % width;
            y = i / width;
            color.setColor(label[i] * -4500 - 1);
            pic.drawRect(x, y, x + 2, y + 2, color);
        }//for


        bitsy =  Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        pic = new Canvas(bitsy);
        for(int i = 0; i < pixels.length; i++) {
            x = i % width;
            y = i / width;
            color.setColor(label[i] * -4500 - 1);
            pic.drawRect(x, y, x + 2, y + 2, color);
        }//for

        //shapeProcessor(label, width);

        return bitsy;
    }

    public int[] shapeProcessor (int[] labels, int imageWidth) {

        List<Integer> colours = new ArrayList<Integer>();
        List<Integer> colourRanks = new ArrayList<Integer>();
        List<List<Integer>> colourCoords = new ArrayList<List<Integer>>();
        int[] blobRanks;
        boolean highRank;
        int index;
        int[] centres = new int[5];

        for (int i = 0; i < labels.length; i++) {

            if (labels[i] != -1)
                if (!colours.contains(labels[i])) {

                    colourRanks.add(new Integer(0));
                    colours.add(labels[i]);
                    colourCoords.add(new ArrayList<Integer>());
                    colourCoords.get(colourCoords.size() - 1).add(i);

                } else {

                    colourRanks.set(colours.indexOf(labels[i]), new Integer(colourRanks.get(colours.indexOf(labels[i])) + 1));
                    colourCoords.get(colours.indexOf(labels[i])).add(i);

                }

        }

        blobRanks = new int[5];

        for (int i = 0; i < colours.size(); i++) {

            index = i;
            highRank = false;

            if (i > 4)
                for (int j = 0; j < 5; j++) {

                    if (colourRanks.get(index) > colourRanks.get(blobRanks[j])) {
                        highRank = true;
                        index = j;
                    }

                }
            else
                highRank = true;

            if (highRank) {
                blobRanks[index] = i;
            }

        }



        for (int i = 0; i < blobRanks.length; i++) {

            centres[i] = getCentre(colourCoords.get(blobRanks[i]), imageWidth);
            blobRanks[i] = colours.get(blobRanks[i]);
            Log.i("Colours", " Colour: " + (blobRanks[i] * -4500 - 1) + "Centre: " + centres[i]);

        }

        return blobRanks;

    }

    public int getCentre (List<Integer> pixels, int imageWidth) {

        int x = 0;
        int y = 0;

        for (int i = 0; i < pixels.size(); i++) {

            x += pixels.get(i) % imageWidth;
            y += pixels.get(i) / imageWidth;

        }

        x /= pixels.size();
        y /= pixels.size();

        return (x + (y * imageWidth));

    }

    public Bitmap circleEdges (Bitmap bit) {
        int imageWidth = bit.getWidth();
        int imageHeight = bit.getHeight();
        int[] pixels = new int[imageHeight * imageWidth];
        int[] pixStore = new int[pixels.length];
        int[] newPixels = new int[pixels.length];
        int[] newEdgePixels = new int[pixels.length];
        int[] pixRadii = new int[pixels.length];
        int[] indexes = new int[pixels.length];
        int[] label = new int[pixels.length];
        int radianCount = 0;
        int radius = 1;
        int x;
        int y;
        double tempX;
        double tempY;
        int centre = pixels.length / 2 + (imageWidth / 2);
        double radians;
        boolean doNotUse;
        int newLabel = 1;
        double incRad = Math.PI / (2 * rayIntensity);
        int[][] allRadii = new int[1000][1000];
        int[] pixRadCounts = new int[pixels.length];

        bit.getPixels(pixels, 0, imageWidth, 0, 0, imageWidth, imageHeight);

//        for (int i = 0; i < newPixels.length; i++) {
//            pixStore[i] = -1;
//        }

        newPixels[0] = pixels[centre];

        radians = incRad;

        tempX = Math.cos(radians);
        tempY = Math.sin(radians);

        for (int i = 1; i < pixels.length; i++) {

            x = (int) (tempX * radius);
            y = (int) (tempY * radius);

            pixRadii[i] = radius;

            radius++;

            doNotUse = false;
            if ((centre % imageWidth) + x < 0 || (centre % imageWidth) + x > imageWidth - 1)
                doNotUse = true;
            else if ((centre / imageWidth) + y < 0 || (centre / imageWidth) + y > imageHeight - 1)
                doNotUse = true;

            if (!doNotUse) {
                newPixels[i] = pixels[centre + x + (y * imageWidth)];
                allRadii[radius][radianCount] = i;
                pixRadCounts[i] = radianCount;
            } else {
                radianCount++;
                radians += incRad;

                radius = 1;

                tempX = Math.cos(radians);
                tempY = Math.sin(radians);

            }

        }

        for (int i = 1; i < newPixels.length; i++) {
            if (Math.abs(newPixels[i] - newPixels[i - 1]) > edgeAccuracy) {
                newEdgePixels[i] = Color.BLACK;
            } else {
                newEdgePixels[i] = Color.WHITE;
            }
        }

        label[0] = 1;

        Arrays.fill(pixStore, -1);

        pixStore[centre] = newPixels[0];

        radians = incRad;

        radius = 1;

        tempX = Math.cos(radians);
        tempY = Math.sin(radians);

        for (int i = 1; i < pixels.length; i++) {

            x = (int) (tempX * radius);
            y = (int) (tempY * radius);

            radius++;

            doNotUse = false;
            if ((centre % imageWidth) + x < 0 || (centre % imageWidth) + x > imageWidth - 1)
                doNotUse = true;
            else if ((centre / imageWidth) + y < 0 || (centre / imageWidth) + y > imageHeight - 1)
                doNotUse = true;

            if (!doNotUse) {
                pixStore[centre + x + (y * imageWidth)] = newEdgePixels[i];
            } else {
                radians += incRad;

                radius = 1;

                tempX = Math.cos(radians);
                tempY = Math.sin(radians);

            }

        }


        bit = Bitmap.createBitmap(pixStore, imageWidth, imageHeight, Bitmap.Config.ARGB_8888);

        return bit;


    }

    public Bitmap blobsCircle (Bitmap bit) {
        int imageWidth = bit.getWidth();
        int imageHeight = bit.getHeight();
        int[] pixels = new int[imageHeight * imageWidth];
        int[] switchPixStore = new int[pixels.length];
        int[] switchLabelStore = new int[pixels.length];
        int[] pixStore = new int[pixels.length];
        int[] newPixels = new int[pixels.length];
        int radius = 0;
        int x;
        int y;
        int centre = pixels.length / 2 + 3 * (imageWidth / 4);
        double radians = 0;
        int stageInCycle = 0;
        int cycleNum = 1;
        boolean doNotUse = false;
        int newLabel = 1;
        int borderCount = 0;
        int[] label = new int[pixels.length];
        int[] pixRadii = new int[pixels.length];
        int[] centrePoints = {imageWidth / 4 + (imageWidth * imageHeight / 4), 3 * imageWidth / 4 + (imageWidth * imageHeight / 4),
                imageWidth / 4 + (3 * imageWidth * imageHeight / 4),  3 * imageWidth / 4 + (3 * imageWidth * imageHeight / 4)};

        bit.getPixels(pixels, 0, imageWidth, 0, 0, imageWidth, imageHeight);

        for (int i = 0; i < newPixels.length; i++) {
            pixStore[i] = Color.BLACK;
        }

        for (int j = 0; j < 4; j++) {
            centre = centrePoints[j];

            Arrays.fill(newPixels, 0);
            radius = 0;
            borderCount = 0;

            newPixels[0] = pixels[centre];

            radius++;

            radians = Math.PI / 2 * radius;

            cycleNum = (int) (2 * Math.PI / radians);

            stageInCycle = 0;

            for (int i = 1; i < 50000; i++) {

                pixRadii[i] = radius;

                x = (int) (radius * Math.cos((radians * stageInCycle)));
                y = (int) (radius * Math.sin((radians * stageInCycle)));
                stageInCycle++;

                doNotUse = false;

                if ((centre % imageWidth) + x < 0 || (centre % imageWidth) + x > imageWidth - 1)
                    doNotUse = true;
                else if ((centre / imageWidth) + y < 0 || (centre / imageWidth) + y > imageHeight - 1)
                    doNotUse = true;
                else if (pixels[centre + x + (y * imageWidth)] == Color.BLACK) {
                    borderCount++;
                }

                if (borderCount >= 500)
                    break;

                //            if (Math.abs(x) > imageWidth / 2 - 1
                //                    || Math.abs(y) > imageHeight / 2 - 1)
                //                doNotUse = true;

                if (!doNotUse) {
                    newPixels[i] = pixels[centre + x + (y * imageWidth)];
                }


                if (stageInCycle >= cycleNum) {
                    borderCount = 0;

                    radius++;

                    radians = Math.PI / (2 * radius);

                    cycleNum = (int) radius * 4;

                    stageInCycle = 0;
                }

            }

            pixels[centre] = newPixels[0];

            radius = 1;

            radians = Math.PI / 2 * radius;

            cycleNum = radius * 4;

            stageInCycle = 0;

            for (int i = 1; i < pixels.length; i++) {

                x = (int) (radius * Math.cos((radians * stageInCycle) - Math.PI / 38));
                y = (int) (radius * Math.sin((radians * stageInCycle) - Math.PI / 38));

                stageInCycle++;

                doNotUse = false;

                if ((centre % imageWidth) + x < 0 || (centre % imageWidth) + x > imageWidth - 1)
                    doNotUse = true;
                else if ((centre / imageWidth) + y < 0 || (centre / imageWidth) + y > imageHeight - 1)
                    doNotUse = true;

                //            if (Math.abs(x) > imageWidth / 2 - 1
                //                    || Math.abs(y) > imageHeight / 2 - 1)
                //                doNotUse = true;

                if (!doNotUse && newPixels[i] != 0)
                    pixStore[centre + x + (y * imageWidth)] = newPixels[i];

                if (stageInCycle > cycleNum) {
                    radius++;

                    radians = Math.PI / (2 * radius);

                    cycleNum = (int) (2 * Math.PI / radians);

                    stageInCycle = 0;
                }

            }

        }

        bit = Bitmap.createBitmap(pixStore, imageWidth, imageHeight, Bitmap.Config.ARGB_8888);

        return bit;

    }

    public Bitmap multiCircle (Bitmap bit) {
        int imageWidth = bit.getWidth();
        int imageHeight = bit.getHeight();
        int[] pixels = new int[imageHeight * imageWidth];
        int[] pixStore = new int[pixels.length];
        int radius = 1;
        int x;
        int y;
        double cosVal;
        double sinVal;
        int centre = pixels.length / 2 + (imageWidth / 2);
        double radians;
        boolean doNotUse;
        double incRad = Math.PI / (2 * rayIntensity);
        int[] centrePoints = new int[numCentrePoints * numCentrePoints];
        int[] colours = new int[numCentrePoints * numCentrePoints];
        int centreCounter = 0;
        int borderCount = 0;
        int storeCentre = 0;
        int alreadySwitched;
        int[] blobPixels;
        int reflected = 0;
        int oldRadius = 0;
        int centreX;
        int centreY;
        int superCentre = (imageHeight % 2 == 0)? pixels.length / 2 + imageWidth / 2 : pixels.length / 2;
        boolean toBeNeg;

        bit.getPixels(pixels, 0, imageWidth, 0, 0, imageWidth, imageHeight);

//        centrePoints = spiralCentrePoints(superCentre, imageWidth, imageHeight);

        if (numCentrePoints > 1)
            for (int j = 0; j < numCentrePoints; j++) {
                for (int i = 0; i < numCentrePoints; i++) {
                    centrePoints[centreCounter] = pixels.length / 2 + ((i - numCentrePoints / 2) * imageWidth / numCentrePoints) + ((j - numCentrePoints / 2) * imageWidth * imageHeight / numCentrePoints);
                    centreCounter++;
                }
            }
        else
            centrePoints[0] = pixels.length / 2 + imageWidth / 2;

        Arrays.fill(pixStore, -1);

        for (int j = 0; j < centrePoints.length; j++) {
            centre = centrePoints[j];
            colours[j] = (j + 1) * -4500 - 1;
            blobPixels = new int[pixels.length];
            alreadySwitched = 0;
            reflected = 0;
            oldRadius = 0;
            storeCentre = centrePoints[j];

            if (pixStore[centre] != -1) {
                colours[j] = pixStore[j];
                alreadySwitched++;
            }

            radians = incRad;

            cosVal = Math.cos(radians);
            sinVal = Math.sin(radians);

            for (int i = 1; i < pixels.length; i++) {

                x = (int) (cosVal * radius);
                y = (int) (sinVal * radius);

                radius++;

                doNotUse = false;
                if ((centre % imageWidth) + x < 0 || (centre % imageWidth) + x > imageWidth - 1)
//                    if (reflected < reflectLimit) {
//
//                        cosVal = Math.cos(-Math.PI / 2);
//                        sinVal = Math.sin(-Math.PI / 2);
//
//                        radius = 1;
//
//                        centre = centre + x + (y * imageWidth);
//                        x = 0;
//                        y = 0;
//
//                        reflected++;
//
//                    } else
                        doNotUse = true;
                else if ((centre / imageWidth) + y < 0 || (centre / imageWidth) + y > imageHeight - 1)
//                    if (reflected < reflectLimit) {
//
//                        cosVal = Math.cos(-Math.PI / 2);
//                        sinVal = Math.sin(-Math.PI / 2);
//
//                        radius = 1;
//
//                        centre = centre + x + (y * imageWidth);
//                        x = 0;
//                        y = 0;
//
//                        reflected++;
//
//                    } else
                        doNotUse = true;
                else if (pixels[centre + x + (y * imageWidth)] == Color.BLACK) {
                    if (reflected < reflectLimit) {

                        sinVal *= -1;

                        radius = 1;

                        centre = centre + x + (y * imageWidth);
                        x = 0;
                        y = 0;

                        reflected++;
                            
                    } else
                        borderCount++;
                } else if (pixStore[centre + x + (y * imageWidth)] != -1
                        && pixStore[centre + x + (y * imageWidth)] != colours[j]) {
                    doNotUse = true;
                    if (alreadySwitched < switchLimit) {
                        colours[j] = pixStore[centre + x + (y * imageWidth)];
                    }

                    alreadySwitched++;
                }
//                else if (oldRadius > 0 && radius - oldRadius > 100)
//                    doNotUse = true;

                if (!doNotUse && borderCount < borderLimit) {
                    blobPixels[i] = centre + x + (y * imageWidth);
                } else {
                    borderCount = 0;
                    radians += incRad;

                    centre = storeCentre;
                    reflected = 0;

                    oldRadius = radius;
                    radius = 1;

                    cosVal = Math.cos(radians);
                    sinVal = Math.sin(radians);
                }

                if (radians > 2 * Math.PI) {
                    blobPixels[i + 1] = -2;
                    break;
                }

            }

            pixStore[centre] = colours[j];

            for (int i = 0; i < blobPixels.length; i++) {

                if (blobPixels[i] == -2)
                    break;

                pixStore[blobPixels[i]] = colours[j];

            }

        }

        bit = Bitmap.createBitmap(pixStore, imageWidth, imageHeight, Bitmap.Config.ARGB_8888);

        return bit;

    }

    public int[] spiralCentrePoints (int superCentre, int imageWidth, int imageHeight) {

        int centreX;
        int centreY;
        boolean toBeNeg;
        int centreCounter = 0;
        int[] centrePoints = new int[numCentrePoints * numCentrePoints];

        centrePoints[0] = superCentre;

        for (int centreRadius = 1; centreRadius < (numCentrePoints + 1) / 2; centreRadius++) {
            int cycleNum = centreRadius * 8;
            int squareLength = (centreRadius * 2) + 1;

            for (int i = 0; i < cycleNum; i++) {

                toBeNeg = (i >= cycleNum / 2)? true : false;

                centreX = i % (cycleNum / 2);

                if (centreX < squareLength / 2) {
                    centreX = centreX + 1;
                } else if (centreX < squareLength + squareLength / 2 - 1) {
                    centreX = centreRadius;
                } else {
                    centreX = (cycleNum / 2) - centreX - 1;
                }

                if (toBeNeg) {
                    centreX *= -1;
                }

                centreY = i - cycleNum / 4;


                toBeNeg = (centreY < 0 || centreY >= cycleNum / 2);

                centreY += (centreY < 0)? cycleNum / 2 : 0;

                centreY = centreY % (cycleNum / 2);

                if (centreY < squareLength / 2) {
                    centreY = centreY + 1;
                } else if (centreY < squareLength + squareLength / 2 - 1) {
                    centreY = centreRadius;
                } else {
                    centreY = (cycleNum / 2) - centreY - 1;
                }

                if (toBeNeg) {
                    centreY *= -1;
                }

                centreCounter++;
                centrePoints[centreCounter] = superCentre + (centreX * (imageWidth / numCentrePoints)) + (centreY * imageWidth * (imageHeight / numCentrePoints));

            }

        }

        return centrePoints;
    }

}