package org.firstinspires.ftc.teamcode.opmodes.obsolete;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotlib.robot.MecanumHardwareMap;

//@Autonomous(name="Autonomous Basic Build Zone", group="Auto")
public class AutonomousBasicBuildZone extends LinearOpMode
{
    private MecanumHardwareMap robotHardware;
    private ElapsedTime elapsedTime;

    @Override
    public void runOpMode()  {
        robotHardware = new MecanumHardwareMap(this.hardwareMap);
        elapsedTime = new ElapsedTime();

        robotHardware.servoManager.reset();

        update();

        waitForStart();

        telemetry.addData("Status", "Moving Forward");
        robotMove(0, 1, 0, 20);
        robotHardware.servoManager.setPosition(0.0);
        robotMove(45, 1, 0, 120);
    }

    // Handles the drivetrain functions to set the 4 essential variables for movement
    private void robotMove(double course, double velocity, double rotation, double distance)
    {
        robotHardware.drivetrain.setCourse(course * Math.PI/180); //converts a degree input into radians
        robotHardware.drivetrain.setVelocity(velocity); //quarters the velocity since a high velocity causes massive drift following a move command
        robotHardware.drivetrain.setRotation(rotation);
        robotHardware.drivetrain.setTargetPosition(distance * robotHardware.motorTicksPerInch); // adjust a distance in inches to the appropriate amount of motor ticks
        update();
        sleep(1000);
    }

    private void update()
    {
        /*telemetry.addData("Ticks Per IN", robot.motorTicksPerIN);
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
        telemetry.update();*/
        telemetry.update();
    }
}
