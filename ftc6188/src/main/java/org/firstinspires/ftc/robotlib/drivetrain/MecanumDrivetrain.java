package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
/*
The actual final drivetrain used for the robot, just changes the wheel angles and the wheel coefficient math to match our setup
 */
public class MecanumDrivetrain extends HolonomicFourWheelDrivetrain
{
    public MecanumDrivetrain(DcMotor[] motorList, boolean teleOpMode)
    {
        super(motorList, new double[] {-3*Math.PI/4, 3*Math.PI/4, -Math.PI/4, Math.PI/4}, teleOpMode);
    }

    // this math returns a multiplier to the holonomic four wheel drivetrain, the math is very complex but it just works so it shouldnt be changed
    @Override
    double calculateWheelCoefficient(double course, double wheelAngle)
    {
        return (Math.cos(course)-Math.sin(course)/Math.tan(wheelAngle))*Math.signum(wheelAngle);
    }
}
