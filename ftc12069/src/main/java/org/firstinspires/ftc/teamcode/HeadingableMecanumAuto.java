package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.controller.PIDController;
import org.firstinspires.ftc.robotlib.robot.HeadingableMecanumHardwareMap;

@Autonomous (name="Headingable Mecanum Auto", group="Headingable")
public class HeadingableMecanumAuto extends LinearOpMode
{
    private HeadingableMecanumHardwareMap robotHardware;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robotHardware = new HeadingableMecanumHardwareMap(this.hardwareMap);

        while (!robotHardware.imu.isGyroCalibrated());
        waitForStart();

        robotHardware.drivetrain.setTargetHeading(Math.PI/2);
        while(robotHardware.drivetrain.isRotating())
        {
            robotHardware.drivetrain.updateHeading();
            doTelemetry();
        }
        sleep(1000);

        robotHardware.drivetrain.setTargetHeading(-Math.PI/2);
        while(robotHardware.drivetrain.isRotating())
        {
            robotHardware.drivetrain.updateHeading();
            telemetry.addData("Heading", robotHardware.drivetrain.getCurrentHeading());
            telemetry.update();
        }
        sleep(1000);

        while (opModeIsActive())
        {
            robotHardware.drivetrain.updateHeading();
        }
    }

    private void doTelemetry()
    {
        PIDController pid = (PIDController) robotHardware.drivetrain.controller.algorithm;
        telemetry.addData("Heading, Target", robotHardware.drivetrain.controller.getSensorValue() + "," + pid.getTarget());
        telemetry.addData("KP", pid.getKP());
        telemetry.addData("KI", pid.getKI());
        telemetry.addData("KD", pid.getKD());
        telemetry.addData("error", pid.getError());
        telemetry.addData("integral", pid.getIntegral());
        telemetry.addData("derivative", pid.getDerivative());
        telemetry.update();
    }
}
