package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;
import org.jetbrains.annotations.NotNull;

@Autonomous(name="Mecanum Auto", group="Auto")
public class MecanumAuto extends LinearOpMode
{
    private MecanumRobot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumRobot(this.hardwareMap);
        update();

        waitForStart();

        telemetry.addData("Step", "Forward 12 in");
        robotMove(0, 1, 0, 12);
        telemetry.addData("Step", "Backwards 12 in");
        robotMove(180, 1, 0, 12);
        telemetry.addData("Step", "Right 12 in");
        robotMove(90, 1, 0, 12);
        telemetry.addData("Step", "Left 12 in");
        robotMove(270, 1, 0, 12);
        telemetry.addData("Step", "Rotate 360");
        robotMove(0, 0.75, 360, 0);

        telemetry.addData("Step", "Test stop to counter end rotation");
        robotMove(0, 0, 0, 0);

        /* Move commands
        robotMove(0, 0.5, 0, 1);
        // Latch Servos
        robotMove(180, 0.75, 0 ,2);
        // Unlatch Servos
        robotMove(90, 0.75, 0, 1);
        robotMove(0,0.75,0,2);
        robotMove(270,0.75,0,3);
        robotMove(180,0.75,0,2);
        robotMove(90,0.75,0,48);
         */
    }

    private void robotMove(double course, double velocity, double rotation, double distance)
    {
        robot.drivetrain.setRotation(rotation);
        robot.drivetrain.setCourse(course * Math.PI/180);
        robot.drivetrain.setVelocity(velocity);
        robot.drivetrain.setTargetPosition(distance * robot.motorTicksPerIn);
        update();
        robot.drivetrain.position();
        sleep(1000);
    }

    private void update()
    {
        telemetry.addData("Target Motor Ticks", robot.motorTicksPerIn);
        telemetry.addData("Target Motor Rotations", robot.motorTicksPerIn/robot.drivetrain.getTicksPerUnit());
        telemetry.addData("WheelTarget", robot.drivetrain.wheelTargetPositions[0]);
        telemetry.addData("Course", robot.drivetrain.getCourse());
        telemetry.addData("Velocity", robot.drivetrain.getVelocity());
        telemetry.addData("Rotation", robot.drivetrain.getRotation());
        telemetry.addData("Distance", robot.drivetrain.getTargetPosition());
        telemetry.update();
    }
}
