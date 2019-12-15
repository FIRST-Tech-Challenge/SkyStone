package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotlib.motor.EncoderMotor;

public class HeadingableMecanumDrivetrain extends MecanumDrivetrain implements Headingable
{
    // desired heading
    private double targetHeading = 0;
    private double deltaAngle = 0;

    // rotation
    private double rotationPower = 0;

    // IMU
    private BNO055IMUImpl imu;

    public HeadingableMecanumDrivetrain(EncoderMotor[] motorList, double wheelRadius, double wheelToMotorRatio, BNO055IMUImpl imu)
    {
        super(motorList, wheelRadius, wheelToMotorRatio);
        this.imu = imu;
    }

    @Override
    public void setTargetHeading(double deltaAngle)
    {
        this.targetHeading = getCurrentHeading() + deltaAngle;
        this.deltaAngle = deltaAngle;
    }

    @Override
    public double getCurrentHeading() { return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle; }

    @Override
    public double getTargetHeading() { return targetHeading; }

    public double getTargetDelta() { return deltaAngle; }

    @Override
    public void updateHeading() { }

    @Override
    public void rotate()
    {
        /** Inits the first two rotation angle targets, the second one being a smaller target to increase accuracy **/
        double firstRotationAngle;
        double secondRotationAngle;

        /** Converts the delta angle into two rotation commands, first and second angle **/
        if (getRotation() > 0)
        {
            if (deltaAngle > 10)
            {
                firstRotationAngle = (deltaAngle - 10) + devert(-getCurrentHeading());
                secondRotationAngle = deltaAngle + devert(-getCurrentHeading());
            }
            else
            {
                firstRotationAngle = devert(-getCurrentHeading());
                secondRotationAngle = deltaAngle + devert(-getCurrentHeading());
            }
        }
        else
        {
            if (deltaAngle > 10)
            {
                firstRotationAngle = devert(-(deltaAngle - 10) + devert(-getCurrentHeading()));
                secondRotationAngle = devert(-deltaAngle + devert(-getCurrentHeading()));
            }
            else
            {
                firstRotationAngle = devert(-getCurrentHeading());
                secondRotationAngle = devert(-deltaAngle + devert(-getCurrentHeading()));
            }
        }

        /** Rotates to the first rotation angle target **/
        Double firstA = convert(firstRotationAngle - 5);
        Double firstB = convert(firstRotationAngle + 5);
        setRotation(rotationPower);

        if (Math.abs(firstA - firstB) < 11)
        {
            while (!(firstA < -getCurrentHeading() && -getCurrentHeading() < firstB))
            { updateHeading(); }
        }
        else
        {
            while (!((firstA < -getCurrentHeading() && -getCurrentHeading() < 180)
                || (-180 < -getCurrentHeading() && -getCurrentHeading() < firstB)))
            { updateHeading(); }
        }

        /** Rotates to the second rotation angle target **/
        Double secondA = convert(secondRotationAngle - 5);
        Double secondB = convert(secondRotationAngle + 5);
        setRotation(rotationPower/2);

        if (Math.abs(secondA - secondB) < 11)
        {
            while (!(secondA < -getCurrentHeading() && -getCurrentHeading() < secondB))
            { updateHeading(); }
        }
        else
        {
            while (!((secondA < -getCurrentHeading() && -getCurrentHeading() < 180)
                    || (-180 < -getCurrentHeading() && -getCurrentHeading() < secondB)))
            { updateHeading(); }
        }

        /** Finish rotation by resetting motors **/
        finishRotating();
    }

    @Override
    public boolean isRotating() { return isPositioning(); }

    @Override
    public void finishRotating()
    {
        setVelocity(0);
        setRotation(0);
        for (DcMotor motor : motorList) { motor.setPower(0); }
    }

    private double devert(double angle)
    {
        if (angle < 0)
        {
            angle += 360;
        }
        return angle;
    }

    private double convert(double angle)
    {
        if (angle > 179)
        {
            angle = -(360 - angle);
        }
        else if(angle < -180)
        {
            angle = 360 + angle;
        }
        else if(angle > 360)
        {
            angle = angle - 360;
        }
        return angle;
    }

    public void autoRotate(double targetHeading, double velocity)
    {
        setTargetHeading(targetHeading);
        rotationPower = velocity;
        rotate();
    }
}
