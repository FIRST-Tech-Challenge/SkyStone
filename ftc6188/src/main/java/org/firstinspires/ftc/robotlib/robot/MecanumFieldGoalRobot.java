package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.armsystem.FieldGoalArmSystem;
import org.firstinspires.ftc.robotlib.motor.LimitedMotor;
import org.firstinspires.ftc.robotlib.sound.BasicSound;

public class MecanumFieldGoalRobot extends MecanumRobot
{
    // Motors but now they're a new kind of motor
    private LimitedMotor armVerticalSlide;
    private LimitedMotor armHorizontalSlide;
    public Servo armGripSlide;

    // FieldGoalArmSystem much like a drivetrain
    public FieldGoalArmSystem armSystem;

    // Motor tick limits as found in test
    private static final int[] VERTICAL_LIMITS = {0, 1270};
    private static final int[] HORIZONTAL_LIMITS = {-1400, -400};

    // Sound players
    public BasicSound basicSound;

    public MecanumFieldGoalRobot(HardwareMap hwMap, Telemetry telemetry, boolean teleOpMode)
    {
        super(hwMap, telemetry, teleOpMode);
        // Sound init
        basicSound = new BasicSound("police_siren", hwMap, 0, true);
        basicSound.stopSound();

        // Motors init
        armVerticalSlide = new LimitedMotor(hwMap.get(DcMotor.class, "armVerticalSlide"), VERTICAL_LIMITS[0], VERTICAL_LIMITS[1]);
        armHorizontalSlide = new LimitedMotor(hwMap.get(DcMotor.class, "armHorizontalSlide"), HORIZONTAL_LIMITS[0], HORIZONTAL_LIMITS[1]);
        armGripSlide = hwMap.get(Servo.class, "armGripSlide");

        // Disables the limiting function of the LimitedMotors for testing
        armVerticalSlide.setLimited(true);
        armHorizontalSlide.setLimited(true);

        // Finally initialize the armSystem with the LimitedMotors instead of regular motors
        armSystem = new FieldGoalArmSystem(armVerticalSlide, armHorizontalSlide);
    }

    @Override
    public void informationUpdate()
    {
        telemetry.addData("> Drive Info", "-----");
        telemetry.addData("Half Power Mode(G1-RStickButton)", drivetrain.getLowPower());
        telemetry.addData("Course Degrees", drivetrain.getCourse());
        telemetry.addData("Velocity", drivetrain.getVelocity());
        telemetry.addData("Rotation", drivetrain.getRotation());

        telemetry.addData("> Arm Info", "Limited(G2-B)? " + armVerticalSlide.isLimited());
        telemetry.addData("Vertical Position(G2-LStickY)", armVerticalSlide.getPosition());
        telemetry.addData("Horizontal Position(G2-RStickY", armHorizontalSlide.getPosition());

        telemetry.addData("> Servo Info", "-----");
        telemetry.addData("Platform Servos Pos(G2-DpadUp/DpadDown)", platformServos.getPosition());
        telemetry.addData("Platform Claw Left", "Pos: " + servoClawLeft.getPosition());
        telemetry.addData("Platform Claw Right", "Pos: " + servoClawRight.getPosition());
        telemetry.addData("Arm Grip Slide(G2-Y/A)", armGripSlide.getPosition());

        telemetry.addData("> Sound Info", "Playing(G1-X)? " + basicSound.isSoundPlaying());

        telemetry.update();
    }
}
