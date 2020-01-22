package org.firstinspires.ftc.teamcode.Library;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.LinkedList;
import java.util.Queue;

/* Lena */

public class ColorTools {
    public int hsv_h;
    public int hsv_s;
    public int hsv_v;

    int colorHSVnow;
    int listSize;
    int averageLastHue;
    int total;
    float[] hsvNow;
    Queue<Integer> colorList;
    Queue<Integer> averageList;
    final double SCALE_FACTOR = 255; //Umrechnungsfaktor




    int wertvorxsekunden;
    int x = 2;

    boolean isColorChangend;

    public boolean isColor(ColorEnum color_enum, ColorSensor colorSensor) {
        if(color_enum.equals(ColorEnum.Blue)) return isBlue(colorSensor);
        return isRed(colorSensor);
    }
    /**
     * it returns true if the color is red
     * red is between 0-60° or 330-360° (hue) and with saturation over 0.3
     * @param colorSenseRed is the color sensor we get our values from
     * @return it true or false
     */
    public boolean isRed(ColorSensor colorSenseRed){
        float[] hsvIsRed = showHSV(colorSenseRed);

        if (hsvIsRed[0] >= 0 && hsvIsRed[0] <= 60 && hsvIsRed[1] >= 0.27) {
            return true;
        } else if (hsvIsRed[0] >= 330 && hsvIsRed[0] <= 360){
            return true;
        }

        return false;

    }

    /**
     * it returns true if the color is blue
     * blue is between 120-290° (hue) and with saturation over 0.3
     * @param colorSenseBlue is the color sensor we get our values from
     * @return true or false
     */
    public boolean isBlue(ColorSensor colorSenseBlue) {
        float[] hsvIsBlue = showHSV(colorSenseBlue);

        if (hsvIsBlue[0] >= 160 && hsvIsBlue[0] <= 290){ //[0] 120, && hsvIsBlue[1] >= 2 &&  && hsvIsBlue[2] <= 7
            return true;
        }

        return false;
    }


    /**
     * the boolean gives a true back if the color has changed
     * it calculates the average of the queue and compares it to the average before
     * @param colorSenseChange is the color sensor we get our values from
     * @return true or false
     */

    public boolean colorChange(ColorSensor colorSenseChange, Queue<Integer> colorList) {

        averageList = new LinkedList<>();
        hsvNow = showHSV(colorSenseChange);
        colorHSVnow = (int) hsvNow[0];
        listSize = colorList.size();
        averageLastHue = 0;
        total = 0;
        isColorChangend = false;

                for (int hue: colorList){
                    total = total + hue;
                }

                averageLastHue = total / 100;

                if((averageLastHue < colorHSVnow-10 || averageLastHue > colorHSVnow+10)){
                    isColorChangend = true;
                }


        return isColorChangend;

    }

    /**
     * it converts rgb to hsv-values
     * @param HSVvalues is the color sensor we get the rgb-values from
     * @return it returns hsv-values
     */
    public float[] showHSV (ColorSensor HSVvalues) {
        float[] hsv = new float[3];

        Color.RGBToHSV((int) (HSVvalues.red() * SCALE_FACTOR),
                (int) (HSVvalues.green() * SCALE_FACTOR),
                (int) (HSVvalues.blue() * SCALE_FACTOR),
                hsv);

        return hsv;

    }
}
