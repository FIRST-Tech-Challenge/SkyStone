package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.armsystem.FieldGoalArmSystem;
import org.firstinspires.ftc.robotlib.motor.LimitedMotor;

public class FieldGoalRobot
{
    // Motors/Servos
    private LimitedMotor armVerticalSlide;
    private LimitedMotor armHorizontalSlide;
    public Servo armGripSlide;

    // Systems
    public FieldGoalArmSystem armSystem;

    public FieldGoalRobot(HardwareMap hwMap, Telemetry telemetry, boolean teleOpMode)
    {
        // Collect hardware data
        armVerticalSlide = new LimitedMotor(hwMap.get(DcMotor.class, "armVerticalSlide"), 0, 1270);
        armHorizontalSlide = new LimitedMotor(hwMap.get(DcMotor.class, "armHorizontalSlide"), -1200, 0);
        armGripSlide = hwMap.get(Servo.class, "armGripSlide");

        // Set the upper limits of the LimitedMotors, lower limit is 0
        armVerticalSlide.setUpperLimit(24 * (int)armVerticalSlide.getTicksPerRev());
        armHorizontalSlide.setUpperLimit(12 * (int)armHorizontalSlide.getTicksPerRev());

        // Disables the limiting function of the LimitedMotors for testing
        armVerticalSlide.setLimited(false);
        armHorizontalSlide.setLimited(false);

        // Direction
        armVerticalSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        armHorizontalSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        // Servo fields init
        armGripSlide.setDirection(Servo.Direction.FORWARD);

        // Finally initialize the armSystem with the LimitedMotors instead of regular motors
        armSystem = new FieldGoalArmSystem(armVerticalSlide, armHorizontalSlide);

        // Init telemetry
        telemetry.addLine()
                .addData("Vertical Pos", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + armVerticalSlide.getPosition());
                    }
                })
                .addData("Horizontal Pos", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + armHorizontalSlide.getPosition());
                    }
                });
        telemetry.addLine()
                .addData("Vertical Power", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + armVerticalSlide.getPower());
                    }
                })
                .addData("Horizontal Power", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + armHorizontalSlide.getPower());
                    }
                });
        telemetry.addLine()
                .addData("Servo Position", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("" + armGripSlide.getPosition());
                    }
                });
        telemetry.addLine()
                .addData("Is Limited", new Func<String>()
                {
                    @Override public String value()
                    {
                        return ("Vert: " + armVerticalSlide.isLimited() + " Horiz: " + armHorizontalSlide.isLimited());
                    }
                });
    }
}
