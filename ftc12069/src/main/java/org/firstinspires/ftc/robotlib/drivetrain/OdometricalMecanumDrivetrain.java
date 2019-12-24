package org.firstinspires.ftc.robotlib.drivetrain;

import org.firstinspires.ftc.robotlib.controller.FinishableIntegratedController;

/**
 * A Mecanum Drivetrain with some sort of sensor, such as a gyroscope.
 * Uses the sensor and a controller (PID) specified by the user to control the system.
 */
public class OdometricalMecanumDrivetrain extends MecanumDrivetrain implements Odometrical, Extrinsicable {
    /**
     * The controller being used.
     * @see org.firstinspires.ftc.robotlib.controller.PIDController
     */
    public FinishableIntegratedController controller;

    /**
     * The last time the heading was not within the heading tolerance.
     * @see FinishableIntegratedController
     */
    private long lastOutOfRange;

    /**
     * The target heading
     */
    private double targetHeading = 0;

    /**
     * Whether the drivetrain is operating extrinsically
     */
    private boolean extrinsic;
    private double extrinsicOffset;

    private double course;

    /**
     * The constructor for the drivetrain.
     * @param motorList The array of motors in the drivetrain
     * @param controller Which controller you want the system to use.
     * @see org.firstinspires.ftc.robotlib.controller.PIDController
     */
    public OdometricalMecanumDrivetrain(EncoderMotor[] motorList, FinishableIntegratedController controller)
    {
        super(motorList);
        this.controller = controller;
    }

    /**
     * Set the target heading of the drivetrain.
     * @param targetHeading The angle that you want the drivetrain to move towards
     */
    @Override
    public void setTargetHeading(double targetHeading)
    {
        this.targetHeading = targetHeading;
        controller.setTarget(targetHeading);
    }

    /**
     * Get the current heading of the drivetrain (presumably a value from a sensor and not necessarily the drivetrain's target heading).
     * @return The angle the drivetrain is currently facing
     */
    @Override
    public double getCurrentHeading()
    {
        return controller.getSensorValue();
    }

    /**
     * Get the target heading of the drivetrain (not necessarily the actual, current heading of the drivetrain), passed in using {@link #setTargetHeading}.
     * @return The heading that the robot is currently trying to get to or maintain
     */
    @Override
    public double getTargetHeading()
    {
        return this.targetHeading;
    }

    /**
     * Recalculate motor powers to maintain or move towards the target heading
     */
    @Override
    public void updateHeading()
    {
        controller.update();
        //setRotation(controller.output());
        this.setMotorPowers(this.getWheelRotationValues(controller.output()));
    }

    @Override
    public void rotate()
    {
        while (isRotating())
        {
            updateHeading();
            //updateCourse();
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

    /**
     * Use this as a loop condition (with {@link #updateHeading in the loop body) if you want to turn to a specific heading and then move on to other code.
     * @return Whether or not the drivetrain is still rotating towards the target heading
     */
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
