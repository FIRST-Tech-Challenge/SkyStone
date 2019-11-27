package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumTelemetryDrivetrain extends MecanumDrivetrain
{
    private Telemetry telemetry;

    public MecanumTelemetryDrivetrain(DcMotor[] motorList, boolean teleOpMode, double wheelRadius, double wheelToMotorRatio, Telemetry telemetry)
    {
        super(motorList, teleOpMode, wheelRadius, wheelToMotorRatio);
        this.telemetry = telemetry;
    }

    public void autoPosition(double course, double distanceIN, double velocity, double rotation)
    {
        this.setCourse(course * (Math.PI/180));
        this.setVelocity(velocity);
        this.setRotation(rotation);
        this.setTargetPosition(distanceIN * getTicksPerIn());

        while (isPositioning())
        {
            informationUpdate();
            updatePosition();
        }
        finishPositioning();
    }

    public void informationUpdate()
    {
        telemetry.addData("> Target Positions", "-----");
        telemetry.addData("WheelTarget FL", wheelTargetPositions[0]);
        telemetry.addData("WheelTarget FR", wheelTargetPositions[1]);
        telemetry.addData("WheelTarget RL", wheelTargetPositions[2]);
        telemetry.addData("WheelTarget RR", wheelTargetPositions[3]);
        telemetry.addData("Distance Target", getTargetPosition());

        telemetry.addData("> Wheel Positions", "-----");
        telemetry.addData("WheelPos FL", motorList[0].getCurrentPosition());
        telemetry.addData("WheelPos FR", motorList[1].getCurrentPosition());
        telemetry.addData("WheelPos RL", motorList[2].getCurrentPosition());
        telemetry.addData("WheelPos RR", motorList[3].getCurrentPosition());
        telemetry.addData("Current Pos Percent", getCurrentPosition());
        telemetry.addData("Current Pos", getCurrentPosition() * getTargetPosition());

        telemetry.addData("> Wheel Powers", "-----");
        telemetry.addData("WheelPower FL", motorList[0].getPower());
        telemetry.addData("WheelPower FR", motorList[1].getPower());
        telemetry.addData("WheelPower RL", motorList[2].getPower());
        telemetry.addData("WheelPower RR", motorList[3].getPower());

        telemetry.addData("> Is Busy", "-----");
        telemetry.addData("FL", motorList[0].isBusy());
        telemetry.addData("FR", motorList[1].isBusy());
        telemetry.addData("RL", motorList[2].isBusy());
        telemetry.addData("RR", motorList[3].isBusy());

        telemetry.addData("> Drivetrain Info", "-----");
        telemetry.addData("Course Radians", getCourse());
        telemetry.addData("Course Degrees", getCourse() * Math.PI/180);
        telemetry.addData("Rotation Target", getRotation());
        telemetry.addData("Velocity Target", getVelocity());
        telemetry.addData("Current Pos", getCurrentPosition());
        telemetry.addData("Is Pos", isPositioning());

        telemetry.update();
    }
}
