package org.firstinspires.ftc.teamcode.newhardware.FXTSensors;

import android.util.Log;

import org.firstinspires.ftc.teamcode.RC;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;

/**
 * Created by FIXIT on 15-11-01.
 */
public class I2cSensor implements I2cController.I2cPortReadyCallback {

    public I2cDevice device;
    public I2cAddr i2cReadAddr;
    public I2cAddr i2cWriteAddr;
    public int currentReadAddress;
    public int MAX_BYTE_COUNT = 26;

    public int currentWriteAddress;
    public int writeBufferLength;

    public final static int IDLE = 0;
    public final static int SWITCH_TO_READ = 1;
    public final static int SWITCH_TO_WRITE = 2;
    public final static int READ = 3;
    public final static int WRITE = 4;
    public volatile int deviceStage = IDLE;

    public volatile boolean calledBack = true;
    public volatile boolean waitForWrite = false;
    public volatile boolean waitForRead = false;

    public final static String TAG = "I2C";

    //constructor for device with different slave addresses for r/w
    public I2cSensor (String name, byte i2cReadAddr, byte i2cWriteAddr) {

        device = RC.h.i2cDevice.get(name);
        this.i2cReadAddr = new I2cAddr(i2cReadAddr);
        this.i2cWriteAddr = new I2cAddr(i2cWriteAddr);

    } //I2cSensor

    //constructor for device with only one slave address for r/w
    public I2cSensor (String name, byte i2cAddress) {

        this(name, i2cAddress, i2cAddress);

    } //I2cSensor

    public void waitForCallBack() {
        calledBack = false;

        while (!calledBack) {
            delay(10);
        }//while

    }//waitForCallBack

    //waits for the callback to read in proper data
    public void waitForRead() {
        waitForRead = true;

        while (waitForRead) {
            delay(10);
        }//while

    }//waitForCallBack

    public void waitForWrite() {
        waitForWrite = true;

        while (waitForWrite) {
            delay(10);
        }//while

    }//waitForCallBack

    public void beginCallBack (int newAddress) {

        device.registerForI2cPortReadyCallback(this);

        if (newAddress != currentReadAddress) {
            setRegisterAddress(newAddress);
        }

        device.setI2cPortActionFlag();
        device.writeI2cCacheToController();

        waitForCallBack();
        waitForCallBack();
    }

    public int castByte(byte data) {
        return (int) data & 0xFF;
    }

    public void write (int registerAddress, byte... data) {

        device.copyBufferIntoWriteBuffer(data);

        currentWriteAddress = registerAddress;
        writeBufferLength = data.length;

        deviceStage = SWITCH_TO_WRITE;

        waitForWrite();
    }

    public void write (int registerAddress, int... data) {

        byte[] byteData = new byte[data.length];

        for (int i = 0; i < data.length; i++) {
            byteData[i] = (byte) data[i];
        }

        write(registerAddress, byteData);

    }//write

    public void writeOR(int registerAddress, int... data) {

        byte[] regVals = read(registerAddress, data.length);

        for (int i = 0; i < data.length; i++) {
            data[i] = data[i] | regVals[i];
        }//for

        write(registerAddress, data);
    }//writeOR

    public void writeAND(int registerAddress, int... data) {

        byte[] regVals = read(registerAddress, data.length);

        for (int i = 0; i < data.length; i++) {
            data[i] = data[i] & regVals[i];
        }//for

        write(registerAddress, data);
    }//writeOR

    public byte[] read(int registerAddress, int byteCount) {

        int readOffset = getAddressOffset(registerAddress);

        if (readOffset + byteCount > MAX_BYTE_COUNT || readOffset < 0) {

            Log.i("AdafruitIMU", "Altering read address: " + currentReadAddress + " -> " + registerAddress);
            setRegisterAddress(registerAddress);
            readOffset = 0;
            waitForRead();

        }//if

        byte[] data = new byte[byteCount];

        byte[] transfer = device.getCopyOfReadBuffer();

        for (int i = 0; i < data.length && i + readOffset < transfer.length; i++) {
            data[i] = transfer[i + readOffset];
        }//for

        return data;

    }

    public void setRegisterAddress (int newRegisterAddress) {
        this.currentReadAddress = newRegisterAddress;

        deviceStage = SWITCH_TO_READ;

    }//setRegisterAddress

    public int getAddressOffset(int newAddress) {

        return newAddress - currentReadAddress;

    }

    protected static void delay(long ms){

        try {
            Thread.sleep(ms);
        } catch (InterruptedException e){
            Log.e(TAG, "Exception thrown while Thread sleeping");
        }

    }


    public void portIsReady (int i) {

        device.setI2cPortActionFlag();
        boolean writeFullCache = false;

        device.readI2cCacheFromController(); //reading in latest data!

        //if we're doing nothing, we might as well read
        if (deviceStage == IDLE) {
            deviceStage = SWITCH_TO_READ;
        }//if

        //switching to read mode...
        if (deviceStage == SWITCH_TO_READ) {
            device.enableI2cReadMode(i2cReadAddr, currentReadAddress, MAX_BYTE_COUNT);
            writeFullCache = true;
            deviceStage = READ;
        } else if (deviceStage == READ) {
            waitForRead = false; //notifies all threads that a proper read has been done
        }//elseif

        //switching to write mode
        if (deviceStage == SWITCH_TO_WRITE) {
            device.enableI2cWriteMode(i2cWriteAddr, currentWriteAddress, writeBufferLength);
            writeFullCache = true;
            deviceStage = WRITE;
        } else if (deviceStage == WRITE && device.isI2cPortInWriteMode()) {
            writeFullCache = true;
            waitForWrite = false;
            deviceStage = IDLE;
        }//elseif

        if (writeFullCache) {
            device.writeI2cCacheToController();
        } else {
            device.writeI2cPortFlagOnlyToController();
        }//else

        calledBack = true;

    }


    public I2cDevice getHardwareSensor() {
        return device;
    }

}
