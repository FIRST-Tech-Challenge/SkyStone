package org.firstinspires.ftc.robotlib.sound;

public interface Sound
{
    void playSound();
    void stopSound();
    void setSoundID(String identifier);
    int getSoundID();
    boolean isSoundFilePresent();
    boolean isSoundPlaying();
}
