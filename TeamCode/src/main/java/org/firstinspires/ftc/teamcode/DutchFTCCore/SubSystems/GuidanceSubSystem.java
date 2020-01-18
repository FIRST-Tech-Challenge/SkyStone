package org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems;

public class GuidanceSubSystem extends SubSystem {

    public static GuidanceSubSystem instance;

    @Override
    public void Start () {
        instance = this;
    }

}
