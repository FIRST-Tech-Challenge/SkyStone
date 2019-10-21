package org.firstinspires.ftc.teamcode.newhardware.FXTSensors;

import android.hardware.usb.UsbDevice;

import org.firstinspires.ftc.teamcode.RC;

/**
 * Created by Windows on 2016-07-06.
 */
public abstract class Mouse extends HidBridge {

    public static final int X = 1;
    public static final int Y = 2;
    public static final int LEFT_CLICK = 2;
    public static final int RIGHT_CLICK = 3;
    public static final int SCROLL_CLICK = 4;

    private double deltaX = 0;
    private double deltaY = 0;

    public double absolX = 0;
    public double absolY = 0;

    public Mouse(UsbDevice usb) {
        super(RC.c(), usb);
        startReadingThread();
    }

    public Mouse(int vid, int pid) {
        super(RC.c(), pid, vid);
        super.openDevice();
        super.startReadingThread();
    }

    @Override
    public void onReceiveData(byte[] data) {
        deltaX = data[X];
        deltaY = data[Y];
        onNewData(data);
    }

    public abstract void onNewData(byte[] data);

    public void convertCoords(double angle) {
        double intermediateAngle = (Math.atan2(deltaX, deltaY) + angle) % 360;
        double hyp = Math.hypot(deltaX, deltaY);

        double absolAngle;

        if (intermediateAngle <= 90) {
            absolAngle = 90 - intermediateAngle;
            absolX += Math.cos(absolAngle) * hyp;
            absolY += Math.sin(absolAngle) * hyp;
        } else if (intermediateAngle <= 180) {
            absolAngle = 180 - intermediateAngle;
            absolX += Math.sin(absolAngle) * hyp;
            absolY -= Math.cos(absolAngle) * hyp;
        } else if (intermediateAngle <= 270) {
            absolAngle = 270 - intermediateAngle;
            absolX -= Math.cos(absolAngle) * hyp;
            absolY -= Math.sin(absolAngle) * hyp;
        } else if (intermediateAngle <= 360) {
            absolAngle = 360 - intermediateAngle;
            absolX -= Math.cos(absolAngle) * hyp;
            absolY += Math.sin(absolAngle) * hyp;
        }
    }

}
