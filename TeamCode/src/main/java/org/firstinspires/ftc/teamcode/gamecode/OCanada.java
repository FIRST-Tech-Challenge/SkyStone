package org.firstinspires.ftc.teamcode.gamecode;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;

/**
 * Created by Windows on 2016-02-05.
 */
@TeleOp
@Disabled
public class OCanada extends TeleOpMode {

    MediaPlayer mediaPlayer;

    @Override
    public void initialize() {
        mediaPlayer = MediaPlayer.create(RC.a(), R.raw.can);

    }

    public void start() {
        mediaPlayer.start();
    }

    @Override
    public void stop() {
        super.stop();
        mediaPlayer.release();
    }

    @Override
    public void loopOpMode() {

        if (joy2.buttonRight() && !mediaPlayer.isPlaying()) {
        } else if (joy2.buttonLeft()) {
            if(mediaPlayer.isPlaying()) mediaPlayer.pause();
            mediaPlayer.reset();
        }

    }
}
