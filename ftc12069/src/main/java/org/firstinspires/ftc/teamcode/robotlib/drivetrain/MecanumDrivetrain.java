package org.firstinspires.ftc.teamcode.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrivetrain extends HolonomicFourWheelDrivetrain {
    /**
     * Creates a mecanum drivetrain
     * @param motorList list of motors in the drivetrain
     */
    public MecanumDrivetrain(DcMotor[] motorList)
    {
        super(motorList, new double[] {
                -3*Math.PI/4, 3*Math.PI/4, -Math.PI/4, Math.PI/4
        });
    }

    /**
     * Method for calculating mecanum wheel velocity percentage
     * @param course the angle that you want the robot to move
     * @param wheelAngle the angle of the actual moving part of the wheel
     * @return a number between zero and one, which says what percentage of the speed a wheel should move at. Is then multiplied by the velocity
     */
    @Override
    double calculateWheelCoefficient(double course, double wheelAngle)
    {
        return (Math.cos(course)-Math.sin(course)/Math.tan(wheelAngle))*Math.signum(wheelAngle);
    }
}
