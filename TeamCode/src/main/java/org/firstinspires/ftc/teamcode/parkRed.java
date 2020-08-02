package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class parkRed extends LinearOpMode {
    AutonomousMethods bot;

    public void runOpMode() throws InterruptedException {


    bot = new AutonomousMethods(DcMotor.RunMode.RUN_USING_ENCODER, hardwareMap);

    bot.locate(false, false, false);
    waitForStart();
    bot.initialDown();
    sleep(2000);
    bot.armIn();
    sleep(1000);
    bot.forward(0.5, 18);
}
}
