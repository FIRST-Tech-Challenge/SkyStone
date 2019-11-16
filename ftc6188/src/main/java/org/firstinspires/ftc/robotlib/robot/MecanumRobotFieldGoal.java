package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.armsystem.FieldGoalArmSystem;
import org.firstinspires.ftc.robotlib.motor.LimitedMotor;

public class MecanumRobotFieldGoal extends MecanumRobot
{
    private LimitedMotor armVerticalSlide;
    private LimitedMotor armHorizontalSlide;

    public FieldGoalArmSystem armSystem;

    private static final int verticalLimit = 24; //in
    private static final int horizontalLimit = 12; //in

    public MecanumRobotFieldGoal(HardwareMap hwMap, Telemetry telemetry, boolean teleOpMode)
    {
        super(hwMap, telemetry, teleOpMode);

        armVerticalSlide = new LimitedMotor(hwMap.get(DcMotor.class, "armVerticalSlide"), 0, 24);
        armHorizontalSlide = new LimitedMotor(hwMap.get(DcMotor.class, "armHorizontalSlide"), 0, 12);

        // Set the upper limits of the LimitedMotors, lower limit is 0
        armVerticalSlide.setUpperLimit(24 * (int)armVerticalSlide.getTicksPerRev());
        armHorizontalSlide.setUpperLimit(12 * (int)armHorizontalSlide.getTicksPerRev());

        // Disables the limiting function of the LimitedMotors for testing
        armVerticalSlide.setLimited(false);
        armHorizontalSlide.setLimited(false);

        // Finally initialize the armSystem with the LimitedMotors instead of regular motors
        armSystem = new FieldGoalArmSystem(armVerticalSlide, armHorizontalSlide);
    }

    @Override
    public void informationUpdate()
    {
        telemetry.addData("> Target Positions", "-----");
        telemetry.addData("WheelTarget FL", drivetrain.motorList[0].getTargetPosition());
        telemetry.addData("WheelTarget FR", drivetrain.motorList[1].getTargetPosition());
        telemetry.addData("WheelTarget RL", drivetrain.motorList[2].getTargetPosition());
        telemetry.addData("WheelTarget RR", drivetrain.motorList[3].getTargetPosition());
        telemetry.addData("Distance Target", drivetrain.getTargetPosition());

        telemetry.addData("> Wheel Positions", "-----");
        telemetry.addData("WheelPos FL", drivetrain.motorList[0].getCurrentPosition());
        telemetry.addData("WheelPos FR", drivetrain.motorList[1].getCurrentPosition());
        telemetry.addData("WheelPos RL", drivetrain.motorList[2].getCurrentPosition());
        telemetry.addData("WheelPos RR", drivetrain.motorList[3].getCurrentPosition());
        telemetry.addData("Current Pos", drivetrain.getCurrentPosition());

        telemetry.addData("> Wheel Powers", "-----");
        telemetry.addData("WheelPower FL", drivetrain.motorList[0].getPower());
        telemetry.addData("WheelPower FR", drivetrain.motorList[1].getPower());
        telemetry.addData("WheelPower RL", drivetrain.motorList[2].getPower());
        telemetry.addData("WheelPower RR", drivetrain.motorList[3].getPower());

        telemetry.addData("> Drivetrain Info", "-----");
        telemetry.addData("Course Radians", drivetrain.getCourse());
        telemetry.addData("Course Degrees", drivetrain.getCourse() * Math.PI/180);
        telemetry.addData("Rotation Target", drivetrain.getRotation());
        telemetry.addData("Velocity Target", drivetrain.getVelocity());
        telemetry.addData("Movement Velocity", drivetrain.getAutoVelocity());

        telemetry.addData("> Arm Info", "-----");
        telemetry.addData("Vertical Power", armVerticalSlide.getPower());
        telemetry.addData("Horizontal Power", armHorizontalSlide.getPower());
        telemetry.addData("Vertical Position", armVerticalSlide.getPosition());
        telemetry.addData("Horizontal Position", armHorizontalSlide.getPosition());

        telemetry.addData("> Servo Info", "-----");
        telemetry.addData("Servo Pos", "One: " + platformServos.getServoOne().getPosition() + " Two: " + platformServos.getServoTwo().getPosition());
        telemetry.addData("Linked Pos", platformServos.getPosition());
        telemetry.update();
    }
}
