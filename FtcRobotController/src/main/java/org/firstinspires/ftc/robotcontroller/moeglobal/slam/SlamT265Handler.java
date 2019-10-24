package org.firstinspires.ftc.robotcontroller.moeglobal.slam;

import android.hardware.usb.*;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import static android.hardware.usb.UsbConstants.USB_DIR_OUT;
import static android.os.Process.THREAD_PRIORITY_BACKGROUND;
import static android.os.Process.setThreadPriority;
import static org.firstinspires.ftc.robotcontroller.moeglobal.slam.Constants.*;

public class SlamT265Handler {
    private UsbDeviceConnection connection;
    private UsbInterface usbInterface;
    private UsbEndpoint control, outEndpoint2, outEndpoint3, inEndpoint129, inEndpoint130, inEndpoint131;
    private byte[] tempSlam = new byte[104];
    private float[] curPose = new float[3];
    private boolean isRunning = false;


    SlamT265Handler(UsbDevice device) {
        initVariables(device);
    }

    public static UsbDevice getDevice() {
        return SlamUsbHandler.getDevice(T265_VID);
    }

    private void initVariables(UsbDevice device) {
        connection = SlamUsbHandler.getDeviceConnection(device);
        usbInterface = device.getInterface(0);
        loadEndpoints();
    }

    private void loadEndpoints() {
        for (int k = 0; k < usbInterface.getEndpointCount(); k++) {
            UsbEndpoint endpoint = usbInterface.getEndpoint(k);
            boolean outgoing = endpoint.getDirection() == USB_DIR_OUT;
            if (outgoing) {
                switch (endpoint.getAddress()) {
                    case 1:
                        control = endpoint;
                        break;
                    case 2:
                        outEndpoint2 = endpoint;
                        break;
                    case 3:
                        outEndpoint3 = endpoint;
                        break;
                }
            } else {
                switch (endpoint.getAddress()) {
                    case 129:
                        if (UsbConstants.USB_DIR_IN == endpoint.getDirection())
                            inEndpoint129 = endpoint;
                        break;
                    case 130:
                        if (UsbConstants.USB_DIR_IN == endpoint.getDirection())
                            inEndpoint130 = endpoint;
                        break;
                    case 131:
                        if (UsbConstants.USB_DIR_IN == endpoint.getDirection())
                            inEndpoint131 = endpoint;
                        break;
                }
            }
        }
    }

    public void startStream() {
        sendInitCode();
        isRunning = true;
        new Thread(new SlamRunnable()).start();
    }

    private void updateSlam() {
        connection.bulkTransfer(inEndpoint131, tempSlam, 0, tempSlam.length, 100);
        ByteBuffer order = ByteBuffer.wrap(tempSlam, 8, 12).order(ByteOrder.LITTLE_ENDIAN);
        for (int i = 0; i < 3; i++) {
            curPose[i] = order.getFloat();
        }
    }

    private void sendInitCode() {
        connection.claimInterface(usbInterface, true);
        sendCode(SLAM_CONTROL);
        sendCode(POSE_CONTROL);
        sendCode(DEV_START);
    }

    private void sendCode(byte[] arr) {
        byte[] bytes = new byte[8];
        connection.bulkTransfer(outEndpoint2, arr, arr.length, SMALL_TIMEOUT);
        connection.bulkTransfer(inEndpoint130, bytes, 8, 100);
    }

    public void killStream() {
        isRunning = false;
    }

    private void closeConnection() {
        connection.releaseInterface(usbInterface);
        connection.close();
    }

    public float[] getCurPose() {
        return curPose;
    }

    public class SlamRunnable implements Runnable {
        public void run() {
            setThreadPriority(THREAD_PRIORITY_BACKGROUND);
            while (isRunning) {
                updateSlam();
            }
            closeConnection();
        }
    }
}
