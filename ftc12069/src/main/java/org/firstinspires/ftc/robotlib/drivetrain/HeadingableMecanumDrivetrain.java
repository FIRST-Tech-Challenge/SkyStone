package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotlib.controller.FinishableIntegratedController;

public class HeadingableMecanumDrivetrain extends MecanumDrivetrain implements Headingable, Extrinsicable
{
    public FinishableIntegratedController controller;

    private long lastOutOfRange;

    private double targetHeading = 0;
    private double extrinsicOffset;
    private double course;

    private boolean extrinsic;

    public HeadingableMecanumDrivetrain(DcMotor[] motorList, FinishableIntegratedController controller)
    {
        super(motorList);
        this.controller = controller;
    }

    @Override
    public void setTargetHeading(double targetHeading)
    {
        this.targetHeading = targetHeading;
        controller.setTarget(targetHeading);
    }

    @Override
    public double getCurrentHeading()
    {
        return controller.getSensorValue();
    }

    @Override
    public double getTargetHeading()
    {
        return this.targetHeading;
    }

    @Override
    public void updateHeading()
    {
        controller.update();
        setRotation(controller.output());
    }

    @Override
    public void rotate()
    {
        while (isRotating())
        {
            updateHeading();
            updateCourse();
        }
        finishRotating();
    }

    @Override
    public void setCourse(double course)
    {
        this.course = course;
        if (extrinsic)
        {
            super.setCourse(course-getCurrentHeading()+extrinsicOffset);
        }
        else
        {
            super.setCourse(course);
        }
    }

    @Override
    public void updateCourse()
    {
        if (extrinsic)
        {
            super.setCourse(course-getCurrentHeading()+extrinsicOffset);
        }
    }

    @Override
    public void updatePosition()
    {
        super.updatePosition();
        updateCourse();
    }

    @Override
    public boolean isRotating()
    {
        return !controller.finished();
    }

    @Override
    public void finishRotating()
    {

    }

    @Override
    public boolean getExtrinsic()
    {
        return extrinsic;
    }

    @Override
    public void setExtrinsic(boolean extrinsic)
    {
        this.extrinsic = extrinsic;
    }

    @Override
    public void setExtrinsicOffset(double extrinsicOffset)
    {
        this.extrinsicOffset = extrinsicOffset;
    }

    @Override
    public double getExtrinsicOffset()
    {
        return this.extrinsicOffset;
    }
}
