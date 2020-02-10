package org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems;

import org.firstinspires.ftc.teamcode.DutchFTCCore.Robot;
import org.firstinspires.ftc.teamcode.DutchFTCCore.Robotconfig;

public class MovementSubSystem extends SubSystem {

    Robot bot;

    @Override
    public void Start() {
        super.Start();
        bot = Robot.instance;
    }

    @Override
    public void Update() {
        super.Update();
    }

    public void DriveChecks(double speed, double xmov, double ymov, double rotation){

        double stickangle;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        //radius of circle the stick is in
        speed = (Math.hypot(xmov, ymov));

        //inverse tangent calculate angle of stick
        stickangle = Math.atan2(xmov, ymov);

        //twisting the circle of units 45 degrees
        stickangle -= Math.PI / 4;

        frontLeftPower = (speed * Math.cos(stickangle) + rotation);
        frontRightPower = (speed * Math.sin(stickangle) - rotation);
        backLeftPower = (speed * Math.sin(stickangle) + rotation);
        backRightPower = (speed * Math.cos(stickangle) - rotation);

        bot.MotorFrontLeft.setPower(frontLeftPower);
        bot.MotorFrontRight.setPower(frontRightPower);
        bot.MotorBackLeft.setPower(backLeftPower);
        bot.MotorBackRight.setPower(backRightPower);

    }
}
