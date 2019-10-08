package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareBeep;
import org.firstinspires.ftc.teamcode.LibraryUltrasonicDrive;

//@Disabled
@TeleOp(name = "PID Ultrasonic Test", group = "Test")
public class PIDUltrasonicTest extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    HardwareBeep robot = new HardwareBeep();
    LibraryUltrasonicDrive ultrasonicDrive = new LibraryUltrasonicDrive();
    boolean last = false;
    int i = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        ultrasonicDrive.init(robot, telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("waitForStart()", i++);
        telemetry.update();
        ultrasonicDrive.ultrasonicDrive(.4, 999999, 0);

        telemetry.addData("Should have ran turnUltrasonic", "");
        telemetry.update();

    }
}