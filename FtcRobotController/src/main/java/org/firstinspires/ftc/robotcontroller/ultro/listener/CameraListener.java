package org.firstinspires.ftc.robotcontroller.ultro.listener;

import com.vuforia.Image;


public interface CameraListener extends IListener {
    void process(Image image/*, Mat mat*/);
}
