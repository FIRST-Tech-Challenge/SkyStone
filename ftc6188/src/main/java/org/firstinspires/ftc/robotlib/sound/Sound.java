package org.firstinspires.ftc.robotlib.sound;

public interface Sound
{
    void playSound();
    void setSoundID(String identifier);
    int getSoundID();
    boolean isSoundFilePresent();
    boolean isSoundPlaying();
}
