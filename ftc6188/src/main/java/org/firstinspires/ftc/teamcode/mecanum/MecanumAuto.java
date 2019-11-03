package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;

@Autonomous(name="Mecanum Auto V-Final", group="Auto")
public class MecanumAuto extends LinearOpMode
{
    private MecanumRobot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumRobot(this.hardwareMap, false);
        robot.platformServos.setPosition(1); //init servos to up position
        update();

        waitForStart();

        // Move commands
        telemetry.addData("Status", "Move to bridge - left 1 ft");
        robotMove(0, 1, 0, 4);
        robotMove(270, 1, 0, 12);
    }

    // Handles the drivetrain functions to set the 4 essential variables for movement
    private void robotMove(double course, double velocity, double rotation, double distance)
    {
        robot.drivetrain.setCourse(course * Math.PI/180); //converts a degree input into radians
        robot.drivetrain.setVelocity(velocity * 0.25); //quarters the velocity since a high velocity causes massive drift following a move command
        robot.drivetrain.setRotation(rotation);
        robot.drivetrain.setTargetPosition(distance * robot.motorTicksPerIN); // adjust a distance in inches to the appropriate amount of motor ticks
        update();
        sleep(1000);
    }

    private void update()
    {
        telemetry.addData("Ticks Per IN", robot.motorTicksPerIN);
        telemetry.addData("WheelTarget FL", robot.drivetrain.motorList[0].getTargetPosition());
        telemetry.addData("WheelTarget FR", robot.drivetrain.motorList[1].getTargetPosition());
        telemetry.addData("WheelTarget RL", robot.drivetrain.motorList[2].getTargetPosition());
        telemetry.addData("WheelTarget RR", robot.drivetrain.motorList[3].getTargetPosition());
        telemetry.addData("Course", robot.drivetrain.getCourse());
        telemetry.addData("Velocity", robot.drivetrain.getVelocity());
        telemetry.addData("Rotation", robot.drivetrain.getRotation());
        telemetry.addData("Distance", robot.drivetrain.getTargetPosition());
        telemetry.addData("Servo Pos", robot.platformServos.getActual());
        telemetry.addData("Linked Pos", robot.platformServos.getPosition());
        telemetry.update();
    }
}
