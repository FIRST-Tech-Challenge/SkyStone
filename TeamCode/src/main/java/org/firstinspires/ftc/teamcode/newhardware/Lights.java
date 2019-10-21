package org.firstinspires.ftc.teamcode.newhardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.roboticslibrary.TaskHandler;

/**
 * Created by Windows on 2017-03-31.
 */

public class Lights extends DcMotorImpl {

    private String TAG = "LIGHTS";

    private String name;

    private volatile int lightsState = 0;
    private volatile boolean useFlash = false;
    private volatile int flashState = 0;

    private Lights(DcMotor motor) {
        super(motor.getController(), motor.getPortNumber());
    }//Lights

    public Lights(String addr) {
        this(RC.h.dcMotor.get(addr));

        this.name = addr;

        TaskHandler.addLoopedTask("LIGHTS." + name.toUpperCase(), new Runnable() {
            @Override
            public void run() {

                synchronized (Lights.this) {

                    if (!useFlash) {

                        if (lightsState > 0) {
                            blink(lightsState);
                        } else if (lightsState < 0) {
                            fadeOut(Math.abs(lightsState) / 2);
                            fadeIn(Math.abs(lightsState) / 2);
                        } else {
                            Lights.this.setPower(1);
                        }//else

                    } else {

                        if (flashState > 0) {
                            blink(flashState);
                        } else if (flashState < 0) {
                            fadeOut(Math.abs(flashState) / 2);
                            fadeIn(Math.abs(flashState) / 2);
                        } else {
                            Lights.this.setPower(1);
                        }//else

                        flashState = lightsState;
                        useFlash = false;
                    }//else

                }//synchronized

            }//run
        }, 5);
    }//Lights

    /*
    if value = 0, lights are constantly on
    if value > 0, lights are blinking every "value" milliseconds
    if value < 0, lights are fading in and out every "abs(value)" milliseconds
     */
    public void setLightsState (int value) {
        lightsState = value;
    }//setLightsState

    public void flashState(int value) {
        flashState = value;
        useFlash = true;
    }//flashState

    private void blink(final int time) {

        Lights.this.setPower(1);
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            close();
        }//catch
        Lights.this.setPower(0);

        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            close();
        }//catch
    }//blink

    private void fadeIn(final int time) {

        long start = System.currentTimeMillis();

        Lights.this.setPower(0);
        Log.i(TAG, Lights.this.getPower() + "");

        while (System.currentTimeMillis() - start < time) {
            Lights.this.setPower((System.currentTimeMillis() - start) / ((double) time));
        }//while

        Lights.this.setPower(1);

        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            close();
        }//catch
    }//fadeIn

    private void fadeOut(final int time) {
        long start = System.currentTimeMillis();

        Lights.this.setPower(1);
        Log.i(TAG, Lights.this.getPower() + "");

        while (System.currentTimeMillis() - start < time) {
            Lights.this.setPower(1 - (System.currentTimeMillis() - start) / ((double) time));
            Log.i(TAG, Lights.this.getPower() + "");
        }//while

        Lights.this.setPower(0);
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            close();
        }//catch
    }//fadeOut


}//Lights
