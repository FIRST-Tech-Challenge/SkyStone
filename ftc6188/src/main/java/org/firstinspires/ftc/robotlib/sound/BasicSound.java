package org.firstinspires.ftc.robotlib.sound;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.ftccommon.SoundPlayer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

public class BasicSound implements Sound
{
    // Sound playing variables
    private int soundID;
    private boolean soundFilePresent;
    private ToggleBoolean playSound;
    private boolean playing = false;

    //  HardwareMap reference for the appcontext
    private HardwareMap hwMap;

    // Parameters
    private SoundPlayer.PlaySoundParams params = null;

    public BasicSound(String identifier, HardwareMap hwMap)
    {
        this.hwMap = hwMap;

        playSound = new ToggleBoolean(false);
        setSoundID(identifier);
    }

    public BasicSound(String identifier, HardwareMap hwMap, int loopControl)
    {
        this(identifier, hwMap);

        params = new SoundPlayer.PlaySoundParams();
        params.loopControl = loopControl;
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

    @Override
    public void playSound()
    {
        if (!playing)
        {
            playing = true;
            SoundPlayer.getInstance().startPlaying(hwMap.appContext, soundID, params, null, null);
        }
    }

    @Override
    public void stopSound()
    {
        playing = false;
        SoundPlayer.getInstance().stopPlayingAll();
    }

    @Override
    public void setSoundID(String identifier)
    {
        soundID = hwMap.appContext.getResources().getIdentifier(identifier, "raw", hwMap.appContext.getPackageName());
        soundFilePresent = SoundPlayer.getInstance().preload(hwMap.appContext, soundID);
    }

    @Override
    public int getSoundID() { return soundID; }

    @Override
    public boolean isSoundFilePresent() { return soundFilePresent; }

    @Override
    public boolean isSoundPlaying() { return playing; }
}
