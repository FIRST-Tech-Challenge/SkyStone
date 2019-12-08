package org.firstinspires.ftc.robotlib.drivetrain;

import org.firstinspires.ftc.robotlib.motor.EncoderMotor;
import org.firstinspires.ftc.robotlib.state.AutoDirection;

/*
The actual final drivetrain used for the robot, just changes the wheel angles and the wheel coefficient math to match our setup
 */
public class MecanumDrivetrain extends HolonomicFourWheelDrivetrain
{
    public MecanumDrivetrain(EncoderMotor[] motorList, double wheelRadius, double wheelToMotorRatio)
    {
        super(motorList, new double[] {-3*Math.PI/4, 3*Math.PI/4, -Math.PI/4, Math.PI/4});
        setTicksPerIn(wheelRadius, wheelToMotorRatio);
    }

    // this math returns a multiplier to the holonomic four wheel drivetrain, the math is very complex but it just works so it should'nt be changed
    @Override
    double calculateWheelCoefficient(double course, double wheelAngle)
    {
        return (Math.cos(course)-Math.sin(course)/Math.tan(wheelAngle))*Math.signum(wheelAngle);
    }

    // automatically triggers the motors to produce the desired movement (do not use rotation?)
    public void autoPosition(double course, double distanceIN, double velocity, double rotation)
    { //TODO: Test mecanum drive with regular DcMotor again since auto problems only developed when switching to EncoderMotor
        this.setCourse(-course * Math.PI/180);
        this.setVelocity(velocity);
        this.setRotation(rotation);
        this.setTargetPosition(distanceIN * getTicksPerIn());
        this.updateMotorPowers();
        this.position();
    }

    public void autoPosition(AutoDirection course, double distanceIN, double velocity, double rotation)
    { autoPosition(course.getAngle(), distanceIN, velocity, rotation);}
}
