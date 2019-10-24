package org.firstinspires.ftc.robotcontroller.moeglobal.slam;

import android.content.Context;
import android.hardware.usb.UsbDevice;
import com.sun.tools.javac.util.Assert;

import java.io.IOException;
import java.io.InputStream;

import static org.firstinspires.ftc.robotcontroller.moeglobal.slam.Constants.LIZARD_VID;

public class SlamLizardHandler {
    public static byte[] firmware;

    public static void uploadFirmware(UsbDevice device) {
        SlamUsbHandler.uploadByteArray(device, firmware);
    }


    public static UsbDevice getDevice() {
        return SlamUsbHandler.getDevice(LIZARD_VID);
    }

    public static void init(Context context) {
        firmware = getFirmware(context);
    }

    private static byte[] getFirmware(Context context) {
        try {
            InputStream is = context.getAssets().open("firmware/target.mvcmd");
            byte[] res = new byte[is.available()];
            int read = is.read(res);
            Assert.check(read != -1);
            return res;
        } catch (IOException e) {
            e.printStackTrace();
            throw new IllegalStateException("Missing T265 Firmware");
        }
    }
}
