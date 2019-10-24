package org.firstinspires.ftc.robotcontroller.moeglobal.oldslam;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDevice;
import android.util.Log;

import static android.hardware.usb.UsbManager.*;
import static org.firstinspires.ftc.robotcontroller.moeglobal.oldslam.OldUSBController.ACTION_USB_PERMISSION;

public class OldUSBReceiver extends BroadcastReceiver {
    private final OldUSBController controller;
    private String TAG = "USBReceiver";

    public OldUSBReceiver(OldUSBController controller) {
        this.controller = controller;
    }

    @Override
    public void onReceive(Context context, Intent intent) {
        String action = intent.getAction();
        switch (action) {
            case ACTION_USB_PERMISSION:
                synchronized (this) {
                    UsbDevice device = intent.getParcelableExtra(EXTRA_DEVICE);
                    if (intent.getBooleanExtra(EXTRA_PERMISSION_GRANTED, false)) {
                        int vendorId = device.getVendorId();

                        if (vendorId == 999)
                            controller.loadT265fw();
                        else if (vendorId == 32903)
                            controller.openStream();
                    } else {
                        Log.e(TAG, "permission denied for device " + device);
                    }
                }
                break;
            case ACTION_USB_DEVICE_ATTACHED:
                synchronized (this) {
                    UsbDevice device = intent.getParcelableExtra(EXTRA_DEVICE);
                    Log.e(TAG, "in device attached action " + device.getDeviceName());

                    if (device.getVendorId() == 32903)
                        controller.getPermission(device);
                }
                break;
            case ACTION_USB_DEVICE_DETACHED:
                synchronized (this) {
                    controller.killStream();
                }
        }
    }
}

