package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.DutchFTCCore.Robot;
import org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems.GuidanceSubSystem;
import org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems.MovementSubSystem;

@TeleOp( name = "TestOpmode")
public class TestOpMode extends OpMode {
    Robot bot;

    @Override
    public void init() {
            bot = new Robot(this);
            bot.StartIMUSubSystem();
            bot.StartGuidanceSubSystem();
            bot.StartMovementSubSystem();
            bot.MotorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
            bot.MotorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            bot.Calibrate();
    }

    @Override
    public void loop() {
        bot.Update();
        //GuidanceSubSystem.targetAngle = 90;
        MovementSubSystem.yMov = 1;
    }


}
