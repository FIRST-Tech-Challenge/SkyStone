package org.firstinspires.ftc.teamcode.Experimental.AccelerometerDeadReckoning;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class RevXHAccelerometerLocalizer {
    BNO055IMU imu;
    MovementState movementState = new MovementState();

    public RevXHAccelerometerLocalizer (BNO055IMU imu) {
        this.imu = imu;
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }


    public void update() {
        Acceleration a = imu.getLinearAcceleration();
        Velocity v = imu.getVelocity();
        Position p = imu.getPosition();

        movementState = new MovementState(p.x, p.y, p.z, v.xVeloc, v.yVeloc, v.zVeloc, a.xAccel, a.yAccel, a.zAccel);
    }
}
