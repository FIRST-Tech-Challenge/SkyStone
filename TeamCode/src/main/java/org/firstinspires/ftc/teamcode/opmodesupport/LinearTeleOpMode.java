package org.firstinspires.ftc.teamcode.opmodesupport;

import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioTrack;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.roboticslibrary.TaskHandler;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Windows on 2016-11-05.
 */
public abstract class LinearTeleOpMode extends LinearOpMode {

        /**
         * Joystick representing the first gamepad
         * @see com.qualcomm.robotcore.hardware.Gamepad
         */
        protected Joystick joy1;

        /**
         * Joystick representing the second gamepad
         * @see com.qualcomm.robotcore.hardware.Gamepad
         */
        protected Joystick joy2;

        /**
         * List of long numbers. Used for timers
         */
        private List<Long> startNanoTimes = new ArrayList<Long>();

        protected int cycles = 0;

        protected String TAG = "FIX IT";

        protected String dataLogFileName = null;

        @Override
        public void runOpMode() throws InterruptedException {
            joy1 = new Joystick();
            joy2 = new Joystick();

            addTimer();
            RC.setOpMode(this);
            TaskHandler.init();

            initialize();

            try{

            RC.t.addData("OpMode", "Initialization Complete");

            waitForStart();
            clearTimer();
            onStart();

            while (opModeIsActive()){
                joy1.update(gamepad1);
                joy2.update(gamepad2);
                cycles++;
                loopOpMode();
                telemetry.update();
            }
            } finally {

                TaskHandler.removeAllTasks();
                stopOpMode();
            }

        }


        public abstract void initialize();

        public void onStart(){};


        /**
         * Method that runs repeatedly. It updates the gamepads and datalogs any lines or sensors
         */

        public abstract void loopOpMode();

        public abstract void stopOpMode();


        /**
         * AUTOMATIC DATALOGGING METHODS
         */

        /**
         * Method to point Android to what text file to write to
         * @param fileName name of .txt file to write to
         * @param overWrite whether or not to overwrite the data there
         */
        public void setDataLogFile (String fileName, boolean overWrite) {
            RC.t.setDataLogFile(fileName, overWrite);
            this.dataLogFileName = fileName;
        }//setDataLogFile

        public void dataLogData(String key, String data) {
            RC.t.dataLogData(dataLogFileName, key, data);
        }

        public void dataLogData(String data) {
            RC.t.dataLogData(dataLogFileName, data);
        }

        public void dataLogData(String key, double data) {
            RC.t.dataLogData(dataLogFileName, key, data);
        }

        public void dataLogData(double data) {
            RC.t.dataLogData(dataLogFileName, data);
        }

        public void dataLogData(String key, int data) {
            RC.t.dataLogData(dataLogFileName, key, data);
        }

        public void dataLogData(int data) {
            RC.t.dataLogData(dataLogFileName, data);
        }


        /**
         * TIMER METHODS
         */

        /**
         * Adds another timer
         */
        public void addTimer() {
            startNanoTimes.add(System.nanoTime());
        }

        /**
         * Removes the timer at the specified index
         */
        public void removeTimer(int index) {
            startNanoTimes.remove(index);
        }

        /**
         * Resets the first timer
         */
        public void clearTimer() {
            startNanoTimes.set(0, System.nanoTime());
        }

        /**
         * Resets the timer at the specified index
         * @param index index of timer to reset
         */
        public void clearTimer(int index) {
            if (startNanoTimes.size() > index) {
                startNanoTimes.set(index, System.nanoTime());
            } else {
                startNanoTimes.add(System.nanoTime());
            }
        }

        /**
         * Gets the count of the first timer
         * @return time since the first timer has been reset (or was created) in nanoseconds
         */
        public long getNanoSeconds() {
            return System.nanoTime() - startNanoTimes.get(0);
        }

        /**
         * Gets the count of the timer at the specified index
         * @param index index of timer to get count from
         * @return time since the timer has been reset (or was created) in nanoseconds
         */
        public long getNanoSeconds(int index) {

            if (startNanoTimes.size() > index) {
                return System.nanoTime() - startNanoTimes.get(index);
            } else {
                startNanoTimes.add(System.nanoTime());
                return 0;
            }

        }

        /**
         * Gets the count of the first timer
         * @return time since the timer has been reset (or was created) in microseconds
         */
        public long getMicroSeconds() {
            return getNanoSeconds() / 1000;
        }

        /**
         * Gets the count of the timer at the specified index
         * @param index index of the timer to get the count from
         * @return time since the timer has been reset (or was created) in microseconds
         */
        public long getMicroSeconds(int index) {
            return getNanoSeconds(index) / 1000;
        }

        /**
         * Gets the count of the first timer
         * @return time since the timer has been reset (or was created) in milliseconds
         */
        public int getMilliSeconds() {
            return (int) (getNanoSeconds() / 1000000);
        }

        /**
         * Gets the count of the timer at the specified index
         * @param index index of the timer to get the count from
         * @return time since the timer has been reset (or was created) in milliseconds
         */
        public int getMilliSeconds(int index) {
            return (int) (getNanoSeconds(index) / 1000000);
        }

        /**
         * Gets the count of the timer at the specified index
         * @return time since the timer has been reset (or was created) in seconds
         */
        public int getSeconds() {
            return (int) (getNanoSeconds() / 1000000000);
        }

        /**
         * Gets the count of the timer at the specified index
         * @param index index of the timer to get the count from
         * @return time since the timer has been reset (or was created) in milliseconds
         */
        public int getSeconds(int index) {
            return (int) (getNanoSeconds(index) / 1000000000);
        }


        /**
         * SOUND METHODS
         */


        /**
         * Method for generating a beep. Used by playSound(int freq, int duration)
         * @param freqOfTone frequency of beep to generate
         */
        private static byte[] genTone(int freqOfTone, int numSamples) {

            double[] sample = new double[numSamples];
            byte[] generatedSnd = new byte[2 * numSamples];
            final int sampleRate = 4000;

            // fill out the array
            for (int i = 0; i < numSamples; ++i) {
                sample[i] = Math.sin(2 * Math.PI * i / (sampleRate / freqOfTone));
            }

            // convert to 16 bit pcm sound array
            // assumes the sample buffer is normalised.
            int idx = 0;
            for (double dVal : sample) {
                short val = (short) (dVal * 32767);
                generatedSnd[idx++] = (byte) (val & 0x00ff);
                generatedSnd[idx++] = (byte) ((val & 0xff00) >>> 8);
            }

            return generatedSnd;
        }

        /**
         * Method to easily to play a beep with a defined frequency and duration
         *
         * @param frequency frequency of beep to play
         * @param duration  how long to play the beep (in seconds)
         */
        public static void playSound(final int frequency, double duration) {

            if (duration > 500)
                duration /= 1000;
            else if (duration > 50)
                duration = 3;

            final int sampleRate = 4000;
            final int numSamples = (int) (duration * sampleRate);


            new Thread() {

                @Override
                public void run() {

                    byte[] generatedSnd;

                    AudioTrack audioTrack = new AudioTrack(AudioManager.STREAM_ALARM,
                            sampleRate, AudioFormat.CHANNEL_CONFIGURATION_MONO,
                            AudioFormat.ENCODING_PCM_16BIT, numSamples,
                            AudioTrack.MODE_STATIC);

                    generatedSnd = genTone(frequency, numSamples);

                    audioTrack.write(generatedSnd, 0, numSamples);
                    audioTrack.play();
                }

            }.start();
        }

}
