package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.util.ElapsedTime;

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

    /** ENCODER BASED POSITIONING **/
    public void autoPositionByEncoder(double course, double distanceIN, double velocity)
    {
        // attempt to start with a full reset of program since encoder presents odd cary-over issue
        this.finishPositioning();

        // set all movement variables and finally enter the positioning loop
        this.setCourse(-course * Math.PI/180);
        this.setVelocity(velocity);
        this.setRotation(0);
        this.setTargetPosition(distanceIN * getTicksPerIn());
        this.updateMotorPowers();
        this.position();
    }

    // Alternatives
    public void autoPositionByEncoder(AutoDirection course, double distanceIN, double velocity)
    { autoPositionByEncoder(course.getAngle(), distanceIN, velocity);}

    /** TIME BASED POSITIONING **/
    public void autoPositionByTime(double course, double timeSeconds, double velocity)
    {
        this.setCourse(-course * Math.PI/180);
        this.setVelocity(velocity);
        this.setRotation(0);

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.seconds() < timeSeconds) { updateMotorPowers(); }

        this.setCourse(0);
        this.setVelocity(0);
        this.setRotation(0);
    }

    public void autoPositionByTime(AutoDirection course, double timeSeconds, double velocity)
    { autoPositionByTime(course.getAngle(), timeSeconds, velocity); }
}
