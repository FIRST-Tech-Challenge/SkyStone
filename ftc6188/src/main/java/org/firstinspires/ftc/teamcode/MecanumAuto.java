package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;

@Autonomous(name="Mecanum Auto VTest", group="Auto")
public class MecanumAuto extends LinearOpMode
{
    private MecanumRobot robot;
    private ElapsedTime elapsedTime;

    private double motorRotations; //DcMotor.getMotorType().getTicksPerRev() returns ticks per revolution so multiply by Pi*r

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumRobot(this.hardwareMap);
        elapsedTime = new ElapsedTime();
        motorRotations = (1.0/(Math.PI*robot.gearRatio*robot.wheelRadius*2)); //maybe need to add ticks

        update();
        waitForStart();

        robotMove(0, 0.5, 0, 1);
        // Latch Servos
//        robotMove(180, 0.75, 0 ,2);
//        // Unlatch Servos
//        robotMove(90, 0.75, 0, 1);
//        robotMove(0,0.75,0,2);
//        robotMove(270,0.75,0,3);
//        robotMove(180,0.75,0,2);
//        robotMove(90,0.75,0,48);
    }

    private void robotMove(double course, double velocity, double rotation, double distance)
    {
        robot.drivetrain.setRotation(rotation);
        robot.drivetrain.setCourse(course * Math.PI/180);
        robot.drivetrain.setVelocity(velocity);
        robot.drivetrain.setTargetPosition(distance * motorRotations);
        update();
        robot.drivetrain.position();
        sleep(1000);
    }

    private void update()
    {
        telemetry.addData("TPU", motorRotations);
        telemetry.addData("WheelTarget", robot.drivetrain.wheelTargetPositions[0]);
        telemetry.addData("Motor TPU", robot.drivetrain.motorList[0].getMotorType().getTicksPerRev());
        telemetry.update();
    }
}
