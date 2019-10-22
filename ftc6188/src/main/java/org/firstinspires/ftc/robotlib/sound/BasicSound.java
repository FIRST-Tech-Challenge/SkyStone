package org.firstinspires.ftc.robotlib.sound;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.ftccommon.SoundPlayer;

public class BasicSound implements Sound
{
    private int soundID;
    private boolean soundFilePresent;
    private boolean soundPlaying;

    private SoundPlayer.PlaySoundParams params;
    private HardwareMap hwMap;

    public BasicSound(String identifier, HardwareMap hwMap) { this(identifier, hwMap, 0, false); }

    public BasicSound(String identifier, HardwareMap hwMap, int loopControl, boolean waitForNonLoopingSoundsToFinish)
    {
        this.hwMap = hwMap;
        soundPlaying = false;

        params = new SoundPlayer.PlaySoundParams();
        params.loopControl = loopControl;
        params.waitForNonLoopingSoundsToFinish = waitForNonLoopingSoundsToFinish;

        setSoundID(identifier);
    }

    public void playSound()
    {
        soundPlaying = true;
        SoundPlayer.getInstance().startPlaying(hwMap.appContext, soundID, params, null, new Runnable()
        {
            @Override
            public void run()
            {
                soundPlaying = false;
            }
        });
    }

    public void setSoundID(String identifier)
    {
        soundID = hwMap.appContext.getResources().getIdentifier(identifier, "raw", hwMap.appContext.getPackageName());
        soundFilePresent = SoundPlayer.getInstance().preload(hwMap.appContext, soundID);
    }

    public int getSoundID(){ return soundID; }

    public boolean isSoundFilePresent() { return soundFilePresent; }

    public boolean isSoundPlaying() { return soundPlaying; }
}
