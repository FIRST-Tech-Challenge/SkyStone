package org.firstinspires.ftc.teamcode.examples;

import android.speech.tts.TextToSpeech;
import android.util.Log;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

/**
 * Created by Alec Krawciw on 2017-08-27.
 */

public class SpeechDemo extends AutoOpMode implements TextToSpeech.OnInitListener {
    TextToSpeech talk;

    @Override
    public void runOp() throws InterruptedException {
        talk = new TextToSpeech(RC.c(), this);


        waitForStart();

        talk.speak("Hello", TextToSpeech.QUEUE_ADD, null);
    }

    @Override
    public void onInit(int status) {
        Log.i("Speaking", "Initialized");
    }
}
