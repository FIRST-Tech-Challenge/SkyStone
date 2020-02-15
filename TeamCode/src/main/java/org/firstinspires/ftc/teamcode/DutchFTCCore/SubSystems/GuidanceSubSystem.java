package org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems;

import org.firstinspires.ftc.teamcode.DutchFTCCore.Robotconfig;

public class GuidanceSubSystem extends SubSystem {

    public static GuidanceSubSystem instance;

    //targets, set by external subsystems
    public static double targetAngle;
    public static double targetxPos;
    public static double targetyPos;

    //output, read by external subsystems
    public static double angleMovement;

    //angle data, don't change
    double angleErrorPrev;
    double angleIntegral;
    double lastAngleTime;
    @Override
    public void Start () {
        instance = this;
    }

    @Override
    public void Update(){
        super.Update();
        if (Robotconfig.angleControl){
            angleControlPID();
            if(MovementSubSystem.instance != null){
                MovementSubSystem.rotation = angleMovement;
            }
        }
    }

    void angleControlPID () {
        double currAngleTime = System.currentTimeMillis();
        double error = targetAngle - IMUSubSystem.currHeading;
        double dt = currAngleTime - lastAngleTime;
        angleIntegral = angleIntegral + error*dt;
        double angleDeriv = (error-angleErrorPrev)/dt;
        angleMovement = Robotconfig.angleKp*error + Robotconfig.angleKi*angleIntegral + Robotconfig.angleKd*angleDeriv;
        angleErrorPrev = error;
        lastAngleTime = currAngleTime;

    }

    //TODO: Add position controlq

}
