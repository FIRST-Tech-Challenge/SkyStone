package org.firstinspires.ftc.teamcode.roboticslibrary;

import android.util.Log;

import org.firstinspires.ftc.robotcontroller.internal.FtcControllerUtils;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

/**
 * Created by FIXIT on 15-08-23.
 */
public class FXTTelemetry {

    public Telemetry telemetry;
    public HashMap<String, DataWriter> out = new HashMap<>();
    private HashMap<Long, Long> speakTimes = new HashMap<>();

    public void setTelemetry (Telemetry telem) {
        this.telemetry = telem;
    }

    public void setDataLogFile(String fileName, boolean overWrite) {
        try {
            out.put(fileName, new DataWriter(fileName, overWrite));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void setDataLogFile(String fileName) {
        setDataLogFile(fileName, true);
    }

    public void close() {
        if (out != null) {

            for (Iterator<Map.Entry<String, DataWriter>> iter = out.entrySet().iterator(); iter.hasNext(); ) {
                String key = iter.next().getKey();

                out.get(key).closeWriter();
                out.remove(key);
            }//for

        }//if
    }//close

    public void close(String key) {
        if (out != null) {

            out.get(key).closeWriter();
            out.remove(key);

        }//if
    }//close

    /*
    AUDIBLE TELEMETRY
     */

    public void speakString(String msg) {
        FtcControllerUtils.speak(msg);
    }//speakString

    public void speakString(String msg, long id) {

        if (speakTimes.containsKey(id) && System.currentTimeMillis() - speakTimes.get(id) > 1000) {
            speakString(msg);
        }//if

        speakTimes.put(id, System.currentTimeMillis());
    }//speakString

    public void beep(int freq, int duration) {
        FtcControllerUtils.speak("beep(" + freq + ", " + duration + ")");
    }//beep

    public void clearAllAudioCommands() {
        FtcControllerUtils.clearAllSpeakCommands();
    }//clearAllAudioCommands

    public void waitUntilSpeakQueueIsEmpty() throws InterruptedException {
        while (!FtcControllerUtils.isSpeakQueueEmpty()) {
            Thread.sleep(100);
            Log.i("FDS", "SDFS");
        }//while
    }//waitUntilSpeakQueueIsEmpty

    public void stopSpeaking() {
        FtcControllerUtils.stopSpeaking();
    }//stopSpeaking

    //methods to quickly telemetry something
    public void addData(String data) {
        telemetry.addData("Data", data);
    }

    public void addData(int data) {
        telemetry.addData("Data", data);
    }

    public void addData(float data) {
        telemetry.addData("Data", data);
    }

    public void addData(double data) {
        telemetry.addData("Data", data);
    }

    public void addData(byte data) {
        telemetry.addData("Data", data);
    }

    //methods to normally telemetry something
    //will only overwrite any uses if a key is used multiple times

    public void addData(String key, String data) {
        telemetry.addData(key, data);
    }

    public void addData(String key, int data) {
        telemetry.addData(key, data);
        Log.i(key, "" + data);
    }

    public void addData(String key, float data) {
        telemetry.addData(key, data);
    }

    public void addData(String key, double data) {
        telemetry.addData(key, data);
    }

    public void addData(String key, byte data) {
        telemetry.addData(key, data);
    }

    public void addData(String key, Object data) {
        telemetry.addData(key, data.toString());
    }


    //One-time data logging primitives

    public void dataLogData(String fileName, String key, String data) {
        telemetry.addData(key, data);

        if (out != null)
            out.get(fileName).write(key + ": " + data);
    }

    public void dataLogData(String fileName, String data) {
        if (out != null)
            out.get(fileName).write(data);
    }

    public void dataLogData(String fileName, String key, double data) {
        telemetry.addData(key, data);

        if (out != null)
            out.get(fileName).write(key + ": " + data);
    }

    public void dataLogData(String fileName, double data) {
        if (out != null)
            out.get(fileName).write(data);
    }

    public void dataLogData(String fileName, String key, int data) {
        telemetry.addData(key, data);

        if (out != null)
            out.get(fileName).write(key + ": " + data);
    }

    public void dataLogData(String fileName, int data) {
        if (out != null)
            out.get(fileName).write(data);
    }

    public void beginDataLogging(final String fileName) {
        TaskHandler.addLoopedTask("FXTTelemetry.DATALOGGING_" + fileName.toUpperCase(), new Runnable() {
            @Override
            public void run() {
                dataLog(fileName);
            }
        });
    }

    public void stopDataLogging(String fileName) {
        out.get(fileName).closeWriter();
        TaskHandler.removeTask("FXTTelemetry.DATALOGGING_" + fileName.toUpperCase());
    }

    public void dataLog(String fileName) {
        //User-defined method
    }//dataLog
}
