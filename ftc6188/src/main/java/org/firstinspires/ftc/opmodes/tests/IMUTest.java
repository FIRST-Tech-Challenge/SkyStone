package org.firstinspires.ftc.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotlib.robot.GyroRobot;

@TeleOp(name="IMU Test", group="Test")
public class IMUTest extends OpMode
{
    private GyroRobot robot;

    @Override
    public void init()
    {
        robot = new GyroRobot(this.hardwareMap, this.telemetry);
    }

    @Override
    public void start()
    {
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    @Override
    public void loop()
    {
        telemetry.update();
        //robot.informationUpdate();
    }

}
