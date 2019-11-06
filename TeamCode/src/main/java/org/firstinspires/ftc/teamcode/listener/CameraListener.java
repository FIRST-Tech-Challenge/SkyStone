package org.firstinspires.ftc.teamcode.listener;

import com.vuforia.Image;

import org.opencv.core.Mat;

public interface CameraListener extends IListener {
    void process(Image image, Mat mat);
}
