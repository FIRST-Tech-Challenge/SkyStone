package org.firstinspires.ftc.robotlib.sound;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.ftccommon.SoundPlayer;

import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

public class BasicSound implements Sound
{
    // Sound playing variables
    private int soundID;
    private boolean soundFilePresent;
    private ToggleBoolean playSound;

    //  HardwareMap reference for the appcontext
    private HardwareMap hwMap;

    public BasicSound(String identifier, HardwareMap hwMap)
    {
        this.hwMap = hwMap;

        playSound = new ToggleBoolean(false);
        setSoundID(identifier);
    }

    public void toggleSound()
    {
        playSound.toggle();
        if (playSound.output()) { playSound(); }
        else { stopSound(); }
    }

    @Override
    public void playSound()
    {
        if (!SoundPlayer.getInstance().isLocalSoundOn())
        {
            SoundPlayer.getInstance().startPlaying(hwMap.appContext, soundID, null, null, new Runnable()
            {
                @Override
                public void run()
                {

                }
            });
        }
    }

    @Override
    public void stopSound() { SoundPlayer.getInstance().stopPlayingAll(); }

    @Override
    public void setSoundID(String identifier)
    {
        soundID = hwMap.appContext.getResources().getIdentifier(identifier, "raw", hwMap.appContext.getPackageName());
        soundFilePresent = SoundPlayer.getInstance().preload(hwMap.appContext, soundID);
    }

    @Override
    public int getSoundID(){ return soundID; }

    @Override
    public boolean isSoundFilePresent() { return soundFilePresent; }

    @Override
    public boolean isSoundPlaying() { return SoundPlayer.getInstance().isLocalSoundOn(); }
}
