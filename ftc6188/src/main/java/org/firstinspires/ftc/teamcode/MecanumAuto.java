package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.robotlib.robot.MecanumRobot;

@Autonomous(name="Mecanum Auto", group="Auto")
public class MecanumAuto extends LinearOpMode
{
    private MecanumRobot robot;
    private MecanumDrivetrain robotDrivetrain;
    private ElapsedTime elapsedTime;

    private double ticksPerUnit; //DcMotor.getMotorType().getTicksPerRev() returns ticks per revolution so multiply by Pi*r

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumRobot(this.hardwareMap);
        robotDrivetrain = new MecanumDrivetrain(robot.motorList);
        elapsedTime = new ElapsedTime();
        ticksPerUnit = robotDrivetrain.getTicksPerUnit()*Math.PI* robot.wheelRadius;

        waitForStart();

        robotMove(0, 1, 0, 12 * ticksPerUnit);
        sleep(1000);
        robotMove(90, 1, 0, 12 * ticksPerUnit);
        sleep(1000);
        robotMove(180, 100, 0, 12 * ticksPerUnit);
    }

    private void robotMove(double course, double velocity, double rotation, double distance)
    {
        robotDrivetrain.setRotation(rotation);
        robotDrivetrain.setCourse(course);
        robotDrivetrain.setVelocity(velocity);
        robotDrivetrain.setTargetPosition(distance);
        robotDrivetrain.position();
    }
}
