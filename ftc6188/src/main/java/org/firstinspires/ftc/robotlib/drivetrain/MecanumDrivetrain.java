package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
The actual final drivetrain used for the robot, just changes the wheel angles and the wheel coefficient math to match our setup
 */
public class MecanumDrivetrain extends HolonomicFourWheelDrivetrain
{
    public MecanumDrivetrain(DcMotor[] motorList, boolean teleOpMode, double wheelRadius, double wheelToMotorRatio)
    {
        super(motorList, new double[] {-3*Math.PI/4, 3*Math.PI/4, -Math.PI/4, Math.PI/4}, teleOpMode);
        setTicksPerIn(wheelRadius, wheelToMotorRatio);
    }

    public MecanumDrivetrain(DcMotor[] motorList, boolean teleOpMode) { this(motorList, teleOpMode, 1, 1); }

    // this math returns a multiplier to the holonomic four wheel drivetrain, the math is very complex but it just works so it shouldnt be changed
    @Override
    double calculateWheelCoefficient(double course, double wheelAngle)
    {
        return (Math.cos(course)-Math.sin(course)/Math.tan(wheelAngle))*Math.signum(wheelAngle);
    }

    public void autoPosition(double course, double distanceIN, double rotation)
    {
        this.setCourse(course * (Math.PI/180));
        this.setVelocity(0);
        this.setRotation(rotation);
        this.setTargetPosition(distanceIN * getTicksPerIn());
        position();
    }
}
