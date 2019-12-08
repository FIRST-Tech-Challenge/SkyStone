package org.firstinspires.ftc.opmodes.mecanum.auto.ParkAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;
import org.firstinspires.ftc.robotlib.state.AutoDirection;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;

@Autonomous(name="Auto Park ALL", group="AutoComp")
public class SiBorgsMecanumAutoPark extends LinearOpMode
{
    // Robot
    private SiBorgsMecanumRobot robot;

    // Fields
    private final double VELOCITY = 0.5;

    // Buttons
    // Attaching capstone buttons
    private Button capstoneOpen;
    private Button capstoneClose;

    // Designating parking path buttons
    private Button moveForward;
    private Button moveRearward;
    private Button moveLeft;
    private Button moveRight;

    // AutoProgram
    private AutoProgram autoPath;

    @Override
    public void runOpMode() throws InterruptedException
    {
        /** INIT **/
        robot = new SiBorgsMecanumRobot(this.hardwareMap, this.telemetry);
        // Button init
        capstoneOpen = new Button();
        capstoneClose = new Button();
        moveForward = new Button();
        moveRearward = new Button();
        moveLeft = new Button();
        moveRight = new Button();

        autoPath = AutoProgram.FORWARD;

        /** Adjustments before the auto program starts **/
        while (!isStarted())
        {
            /** LOADING CAPSTONE **/
            capstoneOpen.input(gamepad1.y || gamepad2.y);
            capstoneClose.input(gamepad1.a || gamepad2.a);

            if (capstoneOpen.onPress()) { robot.armGripSlide.setPosition(ServoState.UP); }
            else if (capstoneClose.onPress()) { robot.armGripSlide.setPosition(ServoState.DOWN); }

            robot.armCrane.setVerticalPower(-gamepad1.left_stick_y);
            robot.armCrane.setHorizontalPower(gamepad1.right_stick_y);

            telemetry.addData("ADD CAPSTONE TO SERVO (G1-DPAD) + (G1-LStick)", robot.armGripSlide.getState());

            /** PROGRAMING AUTO **/
            moveForward.input(gamepad1.dpad_up);
            moveRearward.input(gamepad1.dpad_down);
            moveLeft.input(gamepad1.dpad_left);
            moveRight.input(gamepad1.dpad_right);
            updateProgramPlan();

            telemetry.addData("PROGRAM AUTO", autoPath);
            displayProgramPlan();

            telemetry.update();
            telemetry.clearAll();
        }


        telemetry.addData("RUNNING AUTO PATH", autoPath);
        telemetry.update();
        switch (autoPath)
        {
            case LEFT:
            {
                robot.drivetrain.autoPosition(AutoDirection.FRONT, 1, VELOCITY, 0);
                robot.drivetrain.autoPosition(AutoDirection.LEFT, AutoParkConstants.PARK_SIDE_DIST_IN, VELOCITY, 0);
                break;
            }
            case RIGHT:
            {
                robot.drivetrain.autoPosition(AutoDirection.FRONT, 1, VELOCITY, 0);
                robot.drivetrain.autoPosition(AutoDirection.RIGHT, AutoParkConstants.PARK_SIDE_DIST_IN, VELOCITY, 0);
                break;
            }
            case FRONTRIGHT:
            {
                robot.drivetrain.autoPosition(AutoDirection.FRONT, AutoParkConstants.PARK_FRONT_DIST_IN, VELOCITY, 0);
                robot.drivetrain.autoPosition(AutoDirection.RIGHT, AutoParkConstants.PARK_SIDE_DIST_IN, VELOCITY, 0);
                break;
            }
            case FRONTLEFT:
            {
                robot.drivetrain.autoPosition(AutoDirection.FRONT, AutoParkConstants.PARK_FRONT_DIST_IN, VELOCITY, 0);
                robot.drivetrain.autoPosition(AutoDirection.LEFT, AutoParkConstants.PARK_SIDE_DIST_IN, VELOCITY, 0);
                break;
            }
        }

        sleep(1000);
        requestOpModeStop();
    }

    private void displayProgramPlan()
    {
        switch (autoPath)
        {
            case LEFT:
            {
                telemetry.addData("___", "");
                telemetry.addData("<-_", "");
                telemetry.addData("___", "");
                break;
            }
            case RIGHT:
            {
                telemetry.addData("___", "");
                telemetry.addData("_->", "");
                telemetry.addData("___", "");
                break;
            }
            case FRONTLEFT:
            {
                telemetry.addData("<-_", "");
                telemetry.addData("_|_", "");
                telemetry.addData("___", "");
                break;
            }
            case FRONTRIGHT:
            {
                telemetry.addData("_->", "");
                telemetry.addData("_|_", "");
                telemetry.addData("___", "");
                break;
            }
            case FORWARD:
            {
                telemetry.addData("_^_", "");
                telemetry.addData("_|_", "");
                telemetry.addData("___", "");
                break;
            }
        }
    }

    private void updateProgramPlan()
    {
        switch (autoPath)
        {
            case LEFT:
            {
                if (moveRight.onPress())
                {
                    autoPath = AutoProgram.RIGHT;
                }
                else if (moveForward.onPress())
                {
                    autoPath = AutoProgram.FRONTLEFT;
                }
                break;
            }
            case RIGHT:
            {
                if (moveLeft.onPress())
                {
                    autoPath = AutoProgram.LEFT;
                }
                else if (moveForward.onPress())
                {
                    autoPath = AutoProgram.FRONTRIGHT;
                }
                break;
            }
            case FRONTRIGHT:
            {
                if (moveLeft.onPress())
                {
                    autoPath = AutoProgram.FORWARD;
                }
                else if (moveRearward.onPress())
                {
                    autoPath = AutoProgram.RIGHT;
                }
                break;
            }
            case FRONTLEFT:
            {
                if (moveRight.onPress())
                {
                    autoPath = AutoProgram.FORWARD;
                }
                else if (moveRearward.onPress())
                {
                    autoPath = AutoProgram.LEFT;
                }
                break;
            }
            case FORWARD:
            {
                if (moveRight.onPress())
                {
                    autoPath = AutoProgram.FRONTRIGHT;
                }
                else if (moveLeft.onPress())
                {
                    autoPath = AutoProgram.FRONTLEFT;
                }
                break;
            }
        }
    }
}

enum AutoProgram
{
    FRONTLEFT(),
    FRONTRIGHT(),
    FORWARD(),
    LEFT(),
    RIGHT()
}
