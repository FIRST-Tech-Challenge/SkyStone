package org.firstinspires.ftc.robotcontroller.moeglobal.slam;

import android.content.Context;
import android.hardware.usb.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import static org.firstinspires.ftc.robotcontroller.moeglobal.slam.Constants.CHUNK_SIZE;
import static org.firstinspires.ftc.robotcontroller.moeglobal.slam.Constants.SMALL_TIMEOUT;

public class SlamUsbHandler {
    public static UsbManager usbManager;

    public static void uploadByteArray(UsbDevice device, byte[] arr) {
        UsbDeviceConnection deviceConnection = getDeviceConnection(device);
        UsbInterface usbInterface = getUsbInterface(device);
        UsbEndpoint outgoingEndpoint = getOutgoingEndpoint(usbInterface);
        sendToEndpoint(deviceConnection, usbInterface, outgoingEndpoint, arr);
    }

    private static void sendToEndpoint(UsbDeviceConnection connection, UsbInterface usbInterface, UsbEndpoint outPoint, byte[] arr) {
        if (!connection.claimInterface(usbInterface, true)) {
            throw new IllegalStateException("Failed to claim Usb Interface");
        }

        List<byte[]> listToBeSplit = divideArray(arr, CHUNK_SIZE);
        for (byte[] bytes : listToBeSplit) {
            connection.bulkTransfer(outPoint, bytes, CHUNK_SIZE, SMALL_TIMEOUT);
        }

        connection.releaseInterface(usbInterface);
        connection.close();
    }

    private static List<byte[]> divideArray(byte[] source, int chunkSize) {

        List res = new ArrayList();
        int start = 0;

        for (int i = 0; i < (int) Math.ceil(source.length / (double) chunkSize); i++) {
            res.add(Arrays.copyOfRange(source, start, start + chunkSize));
            start += chunkSize;
        }

        return res;
    }

    private static UsbInterface getUsbInterface(UsbDevice device) {
        return device.getInterface(0);
    }

    private static UsbEndpoint getOutgoingEndpoint(UsbInterface usbInterface) {
        for (int k = 0; k < usbInterface.getEndpointCount(); k++) {
            UsbEndpoint uep = usbInterface.getEndpoint(k);
            if (uep.getDirection() == UsbConstants.USB_DIR_OUT) {
                return uep;
            }
        }
        throw new IllegalStateException("Outgoing endpoint not found");
    }

    public static UsbDeviceConnection getDeviceConnection(UsbDevice device) {
        UsbDeviceConnection connection = usbManager.openDevice(device);
        if (connection == null) throw new IllegalStateException("Failed to Open Device");
        return connection;
    }

    public static UsbDevice getDevice(int vid) {
        HashMap<String, UsbDevice> deviceList = usbManager.getDeviceList();
        for (UsbDevice device : deviceList.values()) {
            if (device.getVendorId() == vid) {
                return device;
            }
        }
        return null;
    }

    static void init(Context context) {
        usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
    }
}
