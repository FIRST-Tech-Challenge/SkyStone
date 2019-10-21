package org.firstinspires.ftc.teamcode.subsystems;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Color;
import android.hardware.SensorEvent;
import android.os.BatteryManager;
import android.speech.tts.TextToSpeech;
import android.view.View;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;

public class Phone {

    private Context context;
    private AndroidGyroscope gyro;
    private double integratedGyroZ;
    private long lastGyroRead;
    private TextToSpeech wordSpeaker;
    private SoundPlayer.PlaySoundParams soundParams;
    private int sounds;
    private View relativeLayout;

    private static Phone instance = null;

    public static synchronized Phone getInstance() {
        return instance != null ? instance : (instance = new Phone());
    }

    private Phone() {}

    public void init(HardwareMap hardwareMap) {
        context = hardwareMap.appContext;
        lastGyroRead = 0;
        integratedGyroZ = 0;
        gyro = new AndroidGyroscope(){
            @Override
            public void onSensorChanged(SensorEvent sensorEvent) {
                super.onSensorChanged(sensorEvent);
                if (lastGyroRead == 0) {
                    lastGyroRead = System.nanoTime();
                    return;
                }
                long time = System.nanoTime() - lastGyroRead;
                lastGyroRead += time;
                integratedGyroZ -= time / 1.0e9 * getY();
            }
        };
        gyro.setAngleUnit(AngleUnit.DEGREES);
        gyro.startListening();
        Activity activity = AppUtil.getInstance().getRootActivity();
        wordSpeaker = new TextToSpeech(activity, null);
        soundParams = new SoundPlayer.PlaySoundParams();
        soundParams.loopControl = 0;
        soundParams.waitForNonLoopingSoundsToFinish = true;
        sounds = 0;
        int relativeLayoutId = hardwareMap.appContext.getResources()
                .getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    }

    // Based on code in Android Docs
    @Deprecated // doesnt make sense on control hub
    public double batteryPct() {

        IntentFilter ifilter = new IntentFilter(Intent.ACTION_BATTERY_CHANGED);
        Intent batteryStatus = context.registerReceiver(null, ifilter);

        double level = batteryStatus.getIntExtra(BatteryManager.EXTRA_LEVEL, -1);
        double scale = batteryStatus.getIntExtra(BatteryManager.EXTRA_SCALE, -1);

        return  100.0 * level / scale;

    }

    public void stopGyro() {
        if (gyro != null) {
            gyro.stopListening();
        }
    }

    public void stopTextToSpeech() {
        if (wordSpeaker != null) {
            wordSpeaker.shutdown();
        }
    }

    public double getGyroAngle() {
        return integratedGyroZ;
    }

    public boolean hasGyro() {
        return gyro != null && gyro.isAvailable();
    }

    @Deprecated // doesnt make sense on control hub
    public void toast(String str, int time) {
        if (time != 0) {
            AppUtil.getInstance().showToast(UILocation.BOTH, str, time);
        } else {
            AppUtil.getInstance().showToast(UILocation.BOTH, str);
        }
    }

    public void queueSoundFile(String fileName) {
        sounds++;
        SoundPlayer.getInstance().startPlaying(context,
                context.getResources().getIdentifier(fileName, "raw", context.getPackageName()), soundParams,
                null,() -> sounds--
        );
    }

    public void queueWordSpeak(String word) {
        wordSpeaker.speak(word, TextToSpeech.QUEUE_ADD, null);
    }

    public void haltSounds() {
        SoundPlayer.getInstance().stopPlayingAll();
        wordSpeaker.stop();
    }

    public boolean hasQueuedSound() {
        return sounds > 0 || wordSpeaker.isSpeaking();
    }

    @Deprecated // doesnt make sense on control hub
    public void setBackgroundColor(int r, int g, int b) {
        relativeLayout.post(() -> {
            relativeLayout.setBackgroundColor(Color.rgb(r, g, b));
        });
    }

    @Deprecated // doesnt make sense on control hub
    public void resetBackgroundColor() {
        relativeLayout.post(() -> {
            relativeLayout.setBackgroundColor(Color.TRANSPARENT);
        });
    }

}
