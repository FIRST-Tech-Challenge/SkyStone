package org.firstinspires.ftc.opmodes.mecanum.auto.ParkAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;
import org.firstinspires.ftc.robotlib.state.AutoDirection;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;

@Autonomous(name="Left Park V-Inside", group="AutoComp")
public class SiBorgsMecanumAutoLeftParkInside extends LinearOpMode
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

        robot.drivetrain.autoPosition(AutoDirection.FRONT, AutoParkConstants.PARK_FRONT_DIST_IN, VELOCITY, 0);
        robot.drivetrain.autoPosition(AutoDirection.LEFT, AutoParkConstants.PARK_SIDE_DIST_IN, VELOCITY, 0);


        // Pause then end the op mode safely
        sleep(1000);
        requestOpModeStop();
    }
}
