package org.firstinspires.ftc.robotcontroller.moeglobal.oldslam;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.*;
import android.os.Handler;
import android.os.Process;
import android.util.Log;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;

public class OldUSBController {

    static final String ACTION_USB_PERMISSION = "com.example.t265lib.USB_PERMISSION";
    private final String TAG = "vladbliat";
    private final BroadcastReceiver mUsbReceiver = new OldUSBReceiver(this);
    private FtcRobotControllerActivity context;
    private UsbManager usbManager;
    private int CHUNK_SIZE = 512 * 32;
    private float[] resPtr;
    private byte[] resBuffer;
    private byte[] msg;
    private byte[] msgResponse;
    private Thread pollingThread;
    private UsbEndpoint inEndpoint131;
    private UsbEndpoint outEndpoint2;
    private UsbEndpoint inEndpoint130;
    private UsbDeviceConnection pollingConnection;
    private UsbInterface t265Interface;
    private boolean isRunning = true;
    private boolean dataStreaming = false;

    public OldUSBController(FtcRobotControllerActivity context, int chunkSize) {
        if (chunkSize != 0) CHUNK_SIZE = chunkSize;
        msg = new byte[9];
        msgResponse = new byte[8];
        resPtr = new float[7];
        this.context = context;
        usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
    }

    public int loadT265fw() {
        UsbDevice ud = getDevice(999);

        if (ud != null) {
//            PendingIntent.getBroadcast(this.context, 0, new Intent(UsbManager.ACTION_USB_DEVICE_ATTACHED), 0);
            IntentFilter filter = new IntentFilter(UsbManager.ACTION_USB_DEVICE_ATTACHED);
            filter.addAction(UsbManager.ACTION_USB_ACCESSORY_DETACHED);
            context.registerReceiver(mUsbReceiver, filter);

            if (!usbManager.hasPermission(ud)) {
                getPermission(ud);
                return -6;
            } else {
                UsbDeviceConnection connection = usbManager.openDevice(ud);

                if (connection == null) return -4;


                int configurationCount = ud.getConfigurationCount();
                UsbEndpoint outEndpoint = null;

                UsbConfiguration usbConfiguration = null;

                usbConfiguration = ud.getConfiguration(0);

                UsbInterface usbInterface = usbConfiguration.getInterface(0);

                for (int k = 0; k < usbInterface.getEndpointCount(); k++) {
                    UsbEndpoint uep = usbInterface.getEndpoint(k);

                    if (UsbConstants.USB_DIR_OUT == uep.getDirection()) {
                        outEndpoint = uep;
                        Log.e("OutEndPointType", String.valueOf(outEndpoint.getType()));
                    }
                }


                byte[] fw = null;
                try {
                    fw = openFirmware();
                } catch (Exception e) {
                    Log.e("vlad", "didn't find the file" + e.getMessage());
                }

                if (connection.claimInterface(ud.getInterface(0), true))
                    Log.e("interfaceClaimed", "Success");
                else
                    Log.e("interfaceClaimed", "Failure");

                List<byte[]> listToBeSplit = divideArray(fw, CHUNK_SIZE);
                for (int i = 0; i < listToBeSplit.size(); i++) {
                    connection.bulkTransfer(outEndpoint, listToBeSplit.get(i), CHUNK_SIZE, 100);
                }

                connection.releaseInterface(ud.getInterface(0));
                connection.close();
            }
        } else {
            return -2;
        }
        return 0;
    }

    public float[] GetPose() {
        return resPtr;
    }

    public int openStream() {
        UsbDevice ud = getDevice(32903);

        if (ud != null) {
            if (!usbManager.hasPermission(ud)) {
                getPermission(ud);
                return -6;
            } else {
                UsbDeviceConnection connection = usbManager.openDevice(ud);
                UsbEndpoint control, outEndpoint3, inEndpoint129;
                control = outEndpoint3 = inEndpoint129 = null;

                if (connection == null) return -3;
                pollingConnection = connection;
                int configurationCount = ud.getConfigurationCount();
                UsbConfiguration usbConfiguration;
                UsbInterface usbInterface = null;


                for (int i = 0; i < configurationCount; i++) {
                    usbConfiguration = ud.getConfiguration(i);

                    for (int j = 0; j < usbConfiguration.getInterfaceCount(); j++) {
                        t265Interface = usbInterface = usbConfiguration.getInterface(j);

                        for (int k = 0; k < usbInterface.getEndpointCount(); k++) {
                            UsbEndpoint uep = usbInterface.getEndpoint(k);
                            int address = uep.getAddress();

                            switch (address) {
                                case 1:
                                    if (UsbConstants.USB_DIR_OUT == uep.getDirection())
                                        control = uep;
                                    break;
                                case 2:
                                    if (UsbConstants.USB_DIR_OUT == uep.getDirection())
                                        outEndpoint2 = uep;
                                    break;
                                case 3:
                                    if (UsbConstants.USB_DIR_OUT == uep.getDirection())
                                        outEndpoint3 = uep;
                                    break;
                                case 129:
                                    if (UsbConstants.USB_DIR_IN == uep.getDirection())
                                        inEndpoint129 = uep;
                                    break;
                                case 130:
                                    if (UsbConstants.USB_DIR_IN == uep.getDirection())
                                        inEndpoint130 = uep;
                                    break;
                                case 131:
                                    if (UsbConstants.USB_DIR_IN == uep.getDirection())
                                        inEndpoint131 = uep;
                                    break;
                            }
                        }
                    }
                }

                connection.claimInterface(usbInterface, true);

                // SLAM_6DOF_CONTROL
                msg[0] = 0x08;
                msg[1] = 0x00;
                msg[2] = 0x00;
                msg[3] = 0x00;
                msg[4] = 0x06;
                msg[5] = 0x10;
                msg[6] = 0x01;
                msg[7] = 0x06;

                int received = -1;

                received = connection.bulkTransfer(outEndpoint2, msg, msg.length, 100);
                Log.e("sent 8 bytes", "received " + received);

                received = connection.bulkTransfer(inEndpoint130, msgResponse, msgResponse.length, 100);
                Log.e("sent 8 bytes", "received " + received);

                // POSE_CONTROL
                msg[0] = 0x09;
                msg[1] = 0;
                msg[2] = 0;
                msg[3] = 0;
                msg[4] = 0x02;
                msg[5] = 0x20;
                msg[6] = 0;
                msg[7] = 0;
                msg[8] = 0;

                received = connection.bulkTransfer(outEndpoint2, msg, msg.length, 1000);
                Log.e("sent 8 bytes", "received " + received);

                received = connection.bulkTransfer(inEndpoint130, msgResponse, msgResponse.length, 100);
                Log.e("sent 8 bytes", "received " + received);

                // DEV_START
                msg[0] = 0x08;
                msg[1] = 0x00;
                msg[2] = 0x00;
                msg[3] = 0x00;
                msg[4] = 0x12;
                msg[5] = 0x00;
                msg[6] = 0x00;
                msg[7] = 0x00;

                received = connection.bulkTransfer(outEndpoint2, msg, msg.length, 100);
                Log.e("sent 8 bytes", "received " + received);

                received = connection.bulkTransfer(inEndpoint130, msgResponse, msgResponse.length, 100);
                Log.e("sent 8 bytes", "received " + received);


             /*   byte[] getTmp = new byte[8];
                getTmp[0] = 0x06;
                getTmp[1] = 0x00;
                getTmp[2] = 0x00;
                getTmp[3] = 0x00;
                getTmp[4] = 0x18;
                getTmp[5] = 0x00;

                byte[] getTmpRes = new byte[512];

                received = connection.bulkTransfer(outEndpoint2, getTmp, getTmp.length, 1000);
                Log.e("sent 8 bytes", "received " + received);

                received = connection.bulkTransfer(inEndpoint130, getTmpRes, getTmpRes.length, 10000);
                Log.e("sent 8 bytes", "received " + received);*/

                resBuffer = new byte[104];
                final Handler handler = new Handler();
                pollingThread = new Thread() {
                    public void run() {
                        final int offset = 8;
                        final byte[] xArray = new byte[4];
                        final byte[] yArray = new byte[4];
                        final byte[] zArray = new byte[4];
                        final byte[] qiArray = new byte[4];
                        final byte[] qjArray = new byte[4];
                        final byte[] qkArray = new byte[4];
                        final byte[] qrArray = new byte[4];

                        // Moves the current Thread into the background
                        Process.setThreadPriority(Process.THREAD_PRIORITY_BACKGROUND);
                        dataStreaming = true;
                        updateUI();
                        while (isRunning) {
                            pollingConnection.bulkTransfer(inEndpoint131, resBuffer, resBuffer.length, 100);
                            System.arraycopy(resBuffer, offset, xArray, 0, xArray.length);
                            System.arraycopy(resBuffer, offset + xArray.length, yArray, 0, yArray.length);
                            System.arraycopy(resBuffer, offset + xArray.length * 2, zArray, 0, zArray.length);
                            System.arraycopy(resBuffer, offset + xArray.length * 3, qiArray, 0, qiArray.length);
                            System.arraycopy(resBuffer, offset + xArray.length * 4, qjArray, 0, qjArray.length);
                            System.arraycopy(resBuffer, offset + xArray.length * 5, qkArray, 0, qkArray.length);
                            System.arraycopy(resBuffer, offset + xArray.length * 6, qrArray, 0, qrArray.length);

                            resPtr[0] = ByteBuffer.wrap(xArray).order(ByteOrder.LITTLE_ENDIAN).getFloat();
                            resPtr[1] = ByteBuffer.wrap(yArray).order(ByteOrder.LITTLE_ENDIAN).getFloat();
                            resPtr[2] = ByteBuffer.wrap(zArray).order(ByteOrder.LITTLE_ENDIAN).getFloat();
                            resPtr[3] = ByteBuffer.wrap(qiArray).order(ByteOrder.LITTLE_ENDIAN).getFloat();
                            resPtr[4] = ByteBuffer.wrap(qjArray).order(ByteOrder.LITTLE_ENDIAN).getFloat();
                            resPtr[5] = ByteBuffer.wrap(qkArray).order(ByteOrder.LITTLE_ENDIAN).getFloat();
                            resPtr[6] = ByteBuffer.wrap(qrArray).order(ByteOrder.LITTLE_ENDIAN).getFloat();

                        }
                        dataStreaming = false;
                        updateUI();

                        pollingConnection.releaseInterface(t265Interface);
                        pollingConnection.close();
                    }
                };
                pollingThread.start();
            }
        } else return -1;

        return 0;
    }

    private void updateUI() {
        context.setStreamingText(dataStreaming);
    }

    public void killStream() {
     /*   byte[] stopMsg = new byte[8];
        stopMsg[0] = 0x06;
        stopMsg[1] = 0x00;
        stopMsg[2] = 0x00;
        stopMsg[3] = 0x00;
        stopMsg[4] = 0x13;
        stopMsg[5] = 0x00;
        byte[] stopMsgRes = new byte[8];

        pollingConnection.bulkTransfer(outEndpoint2, stopMsg, stopMsg.length, 100);
*/
        //int res = pollingConnection.bulkTransfer(inEndpoint130, stopMsgRes, stopMsgRes.length, 100);*/
        isRunning = false;
    }

    private UsbDevice getDevice(int vid) {

        HashMap<String, UsbDevice> deviceList = usbManager.getDeviceList();
        Iterator dg = deviceList.values().iterator();
        UsbDevice ud = null;

        while (dg.hasNext()) {
            UsbDevice arch = (UsbDevice) dg.next();
            if (arch.getVendorId() == vid) ud = arch;
            Log.e(TAG, String.valueOf(arch.getVendorId()));
        }

        return ud;
    }

    private byte[] openFirmware() throws IOException {
        InputStream is = context.getAssets().open("firmware/target.mvcmd");
        byte[] res = new byte[is.available()];
        is.read(res);

        return res;
    }

    private List<byte[]> divideArray(byte[] source, int chunksize) {

        List res = new ArrayList();
        int start = 0;

        for (int i = 0; i < (int) Math.ceil(source.length / (double) chunksize); i++) {
            res.add(Arrays.copyOfRange(source, start, start + chunksize));
            start += chunksize;
        }

        return res;
    }

    private byte[] IntToByteArray(int data) {

        byte[] result = new byte[4];

        result[0] = (byte) ((data & 0xFF000000) >> 24);
        result[1] = (byte) ((data & 0x00FF0000) >> 16);
        result[2] = (byte) ((data & 0x0000FF00) >> 8);
        result[3] = (byte) ((data & 0x000000FF) >> 0);

        return result;
    }

    public void getPermission(UsbDevice usbD) {
        PendingIntent pi;
        if (!usbManager.hasPermission(usbD)) {
            pi = PendingIntent.getBroadcast(this.context, 0, new Intent(ACTION_USB_PERMISSION), 0);
            IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
            context.registerReceiver(mUsbReceiver, filter);
            usbManager.requestPermission(usbD, pi);
        }
    }
//    private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver() {
//        @Override
//        public void onReceive(Context context, Intent intent) {
//            String action = intent.getAction();
//            if (ACTION_USB_PERMISSION.equals(action))
//                synchronized (this) {
//                    UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
//                    if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
//                        int vendorId = device.getVendorId();
//
//                        if (vendorId == 999)
//                            loadT265fw();
//                        else if (vendorId == 32903)
//                            openStream();
//                    } else {
//                        Log.e(TAG, "permission denied for device " + device);
//                    }
//                }
//            if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(action))
//                synchronized (this) {
//                    UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
//                    Log.e(TAG, "in device attached action " + device.getDeviceName());
//
//                    if (device.getVendorId() == 32903)
//                        getPermission(usbManager, device);
//                }
//        }
//    };
}
