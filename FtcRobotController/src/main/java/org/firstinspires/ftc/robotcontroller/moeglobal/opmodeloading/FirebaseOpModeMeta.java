package org.firstinspires.ftc.robotcontroller.moeglobal.opmodeloading;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

public class FirebaseOpModeMeta {
    public String group;
    public boolean isAuton;
    public String qualifiedPath;

    public OpModeMeta getOpModeMeta(String name) {
        return new OpModeMeta(name, isAuton ? OpModeMeta.Flavor.AUTONOMOUS : OpModeMeta.Flavor.TELEOP, group);
    }
}
