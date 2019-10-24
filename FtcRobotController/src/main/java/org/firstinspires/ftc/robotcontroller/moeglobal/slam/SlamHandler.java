package org.firstinspires.ftc.robotcontroller.moeglobal.slam;

import android.content.Context;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;

import static android.hardware.usb.UsbManager.ACTION_USB_DEVICE_ATTACHED;

public class SlamHandler {
    public static SlamT265Handler t265Handler;
    public static IntentFilter USBAddOrRemove = new IntentFilter();
    private static SlamUsbListener usbReceiver = new SlamUsbListener();

    static {
        USBAddOrRemove.addAction(ACTION_USB_DEVICE_ATTACHED);
        USBAddOrRemove.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
    }

    public static void init(Context context) {
        SlamUsbHandler.init(context);
        SlamLizardHandler.init(context);
        checkPreConnection();
    }

    /**
     * checks if devices are already connected
     */
    private static void checkPreConnection() {
        UsbDevice device = SlamLizardHandler.getDevice();
        if (device != null) {
            usbReceiver.handleLizardDeviceAdded(device);
        } else {
            device = SlamT265Handler.getDevice();
            if (device != null) {
                usbReceiver.handleT265DeviceAdded(device);
            }
        }
    }

    public static void initUsbListener(Context context) {
        context.registerReceiver(usbReceiver, USBAddOrRemove);
    }

    public static void unRegisterListener(Context context) {
        context.unregisterReceiver(usbReceiver);
    }
}
