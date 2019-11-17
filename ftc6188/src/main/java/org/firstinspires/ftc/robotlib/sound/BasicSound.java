package org.firstinspires.ftc.robotlib.sound;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.ftccommon.SoundPlayer;

import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

public class BasicSound implements Sound
{
    private int soundID;
    private boolean soundFilePresent;
    private ToggleBoolean playSound;

    private SoundPlayer.PlaySoundParams params;
    private HardwareMap hwMap;

    public BasicSound(String identifier, HardwareMap hwMap) { this(identifier, hwMap, 0, false); }

    public BasicSound(String identifier, HardwareMap hwMap, int loopControl, boolean waitForNonLoopingSoundsToFinish)
    {
        this.hwMap = hwMap;

        params = new SoundPlayer.PlaySoundParams();
        params.loopControl = loopControl;
        params.waitForNonLoopingSoundsToFinish = waitForNonLoopingSoundsToFinish;

        playSound = new ToggleBoolean(false);

        setSoundID(identifier);
    }

    public void playSound()
    {
        stopSound();
        SoundPlayer.getInstance().startPlaying(hwMap.appContext, soundID);
    }

    public void stopSound()
    {
        SoundPlayer.getInstance().stopPlayingAll();
        SoundPlayer.getInstance().stopPlayingLoops();
    }

    public void toggleSound()
    {
        playSound.toggle();
        if (playSound.output())
        {
            playSound();
        }
        else
        {
            stopSound();
        }
    }

    public void setSoundID(String identifier)
    {
        soundID = hwMap.appContext.getResources().getIdentifier(identifier, "raw", hwMap.appContext.getPackageName());
        soundFilePresent = SoundPlayer.getInstance().preload(hwMap.appContext, soundID);
    }

    public int getSoundID(){ return soundID; }

    public boolean isSoundFilePresent() { return soundFilePresent; }

    public boolean isSoundPlaying() { return SoundPlayer.getInstance().isLocalSoundOn(); }
}
