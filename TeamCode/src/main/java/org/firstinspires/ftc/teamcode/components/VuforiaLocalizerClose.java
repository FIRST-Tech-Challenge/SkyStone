package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class VuforiaLocalizerClose extends VuforiaLocalizerImpl {
    public VuforiaLocalizerClose(Parameters parameters) {
        super(parameters);
    }

    public void close() {
        super.close();
    }
}
