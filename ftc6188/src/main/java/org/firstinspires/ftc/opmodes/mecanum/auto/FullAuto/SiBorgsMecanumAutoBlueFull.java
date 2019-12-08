package org.firstinspires.ftc.opmodes.mecanum.auto.FullAuto;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;
import org.firstinspires.ftc.robotlib.state.AutoDirection;

@Disabled
@Autonomous(name="Blue Alliance Full V-CompetitionReady", group="AutoComp")
public class SiBorgsMecanumAutoBlueFull extends LinearOpMode
{
    // Robot
    private SiBorgsMecanumRobot robot;
    private ElapsedTime elapsedTime;

    // Fields
    private static final double VELOCITY = 0.5;

    // Buttons
    private Button capstoneOpen;
    private Button capstoneClose;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new SiBorgsMecanumRobot(this.hardwareMap, this.telemetry);
        robot.changeBackgroundColor(Color.BLUE);

        elapsedTime = new ElapsedTime();
        capstoneOpen = new Button();
        capstoneClose = new Button();

        /** Before the auto period starts the drivers should load a capstone into the arm **/
        while (!isStarted())
        {
            capstoneOpen.input(gamepad1.dpad_up || gamepad2.dpad_down || gamepad1.y || gamepad2.y);
            capstoneClose.input(gamepad1.dpad_down || gamepad2.dpad_down || gamepad1.a || gamepad2.a);

            if (capstoneOpen.onPress()) { robot.armGripSlide.setPosition(ServoState.UP); }
            else if (capstoneClose.onPress()) { robot.armGripSlide.setPosition(ServoState.DOWN); }

            robot.armCrane.setVerticalPower(-gamepad1.left_stick_y);
            robot.armCrane.setHorizontalPower(gamepad1.right_stick_y);

            telemetry.addData("ADD CAPSTONE TO SERVO (G1-DPAD) + (G1-LStick)", robot.armGripSlide.getState());
            telemetry.update();
        }

        telemetry.addData("AUTO START", elapsedTime.seconds());
        telemetry.update();
        /** Auto period now starts **/

        robot.drivetrain.autoPosition(AutoDirection.RIGHT, 32, VELOCITY, 0);
        robot.drivetrain.autoPosition(AutoDirection.FRONT, 4, VELOCITY, 0);

        robot.platformServo.setPosition(ServoState.DOWN);
        sleep(1000);

        robot.drivetrain.autoPosition(AutoDirection.LEFT, 24, VELOCITY, 0);

        robot.platformServo.setPosition(ServoState.UP);
        sleep(1000);

        robot.drivetrain.autoPosition(AutoDirection.REAR, 24, VELOCITY, 0);
        robot.drivetrain.autoPosition(AutoDirection.RIGHT, 20, VELOCITY, 0);
        robot.drivetrain.autoPosition(AutoDirection.FRONT, 24, VELOCITY, 0);
        robot.drivetrain.autoPosition(AutoDirection.LEFT, 36, VELOCITY, 0);
        robot.drivetrain.autoPosition(AutoDirection.REAR, 36, VELOCITY, 0);


        // Pause then end the op mode safely
        sleep(1000);
        requestOpModeStop();
    }
}
