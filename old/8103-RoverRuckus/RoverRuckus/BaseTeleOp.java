package org.firstinspires.ftc.teamcode.RoverRuckus;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Extender;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.SparkyTheRobot;
import org.firstinspires.ftc.teamcode.RoverRuckus.Auto.AutoUtils;
import org.firstinspires.ftc.teamcode.RoverRuckus.Mappings.ControlMapping;
import org.firstinspires.ftc.teamcode.Utilities.Control.FeedbackController;
import org.firstinspires.ftc.teamcode.Utilities.Control.HoldingPIDMotor;
import org.firstinspires.ftc.teamcode.Utilities.Control.LEDRiver;
import org.firstinspires.ftc.teamcode.Utilities.Control.WheelDriveVector;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;

@Config
public abstract class BaseTeleOp extends LinearOpMode {

    // How much current we need to draw before we increase our severity
    public static double SERVO_CURRENT_THRESHOLD = 2800; // 2.8 A
    public static double EXTEND_CURRENT_THRESHOLD = 2000; // 2.0 A

    // How fast the robot moves when the arm is fully extended/retracted
    public static double EXTEND_MAXED_DRIVE_POWER = 0.6;

    // Constants for auto-turning
    public static double TURN_MAX_SPEED = 1.0; // Max auto-turn speed
    public static double TURN_SPEED_CUTOFF = 0.03; // How slow we should turn before stopping
    public static double TURN_CORRECT_FACTOR = 1; // Our P constant for turning

    // How much the robot should turn while strafing
    public static double SLEW_TURN_FACTOR = 0.2;

    // How long the triggers need to be held down before the macros
    // will kick in. They will have to hold down the triggers for this many MS
    // with at least the power stated below
    public static int MS_USE_DOWN_MACROS = 500;
    public static int MS_USE_UP_MACROS = 900;
    public static int MS_OPEN_LATCH = 1200;
    public static double POWER_USE_UP_DOWN_MACROS = 0.3;

    public static int MS_DELAY_EXTEND = 600;

    // How far up the winch should go
    public static int WINCH_MAX_POS = 7700;

    // Where the robot will be facing when the "heading reset" button is clicked
    public static double CRATER_HEADING_RESET = Math.PI * 0.25;
    public static double DEPO_HEADING_RESET = 0;

    public static double CRATER_LANDER_DEPOSIT = Math.PI * 1.75;
    public static double CRATER_HANG = Math.PI * 0.75;
    public static double DEPO_LANDER_DEPOSIT = 0;
    public static double DEPO_HANG = Math.PI * 1.75;

    public static double BLOCK_TRAPPER_TRAPPING = 0.4;
    public static double BLOCK_TRAPPER_PERMISSIVE = 0;

    public static double COUNTER_TURN_FACTOR = -0.04;

    public static double DIR_QUICK_REVERSE = 1;
    int timeStopReversing;

    public static int TIME_GAMEPLAY_DONE = 600;

    public ControlMapping controller;
    public boolean fieldCentric;
    public boolean hangOnCrater;
    public boolean timing;

    public boolean armIsCollecting;
    private boolean endgameActivities;

    private boolean wasTurningTo255;
    private double headingOffset;
    private int winchOffset;

    SparkyTheRobot robot;
    ElapsedTime loopTime;
    ElapsedTime timeMovingArmDown;
    ElapsedTime timeMovingArmUp;
    HoldingPIDMotor winch;
    Extender extender;

    ExpansionHubEx leftHubEx;
    ExpansionHubEx rightHubEx;

    Arm arm;

    @Override
    public void runOpMode() {

        RevExtensions2.init();

        robot = new SparkyTheRobot(this);
        robot.calibrate(true);
        headingOffset = 0;
        winchOffset = 0;


        double desiredHeading;
        ElapsedTime timeUntilLockHeading = new ElapsedTime();


        loopTime = new ElapsedTime();
        timeMovingArmDown = new ElapsedTime();
        timeMovingArmUp = new ElapsedTime();
        wasTurningTo255 = false;
        armIsCollecting = false;
        endgameActivities = false;
        timeStopReversing = 1;

        winch = new HoldingPIDMotor(robot.winch, 1);

        arm = new Arm(robot.leftFlipper, robot.rightFlipper, robot.linearSlide);
        extender = new Extender(robot.slideSwitch, robot.linearSlide);

        // Enable PID control on these motors
        robot.leftFlipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFlipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set up current reading classes
        if (!robot.onRawChassis) {
            leftHubEx = hardwareMap.get(ExpansionHubEx.class, "leftHub");
            rightHubEx = hardwareMap.get(ExpansionHubEx.class, "rightHub");
        }

        // Keep standard front direction
        for (int i = 0; i < 4; i++) {
            if (i % 2 == 1) {robot.motorArr[i].setDirection(DcMotor.Direction.FORWARD);}
            else {robot.motorArr[i].setDirection(DcMotor.Direction.REVERSE);}
        }

        // Display setup readouts
        telemetry.log().clear();
        telemetry.log().add("Running RR2 TeleOp");
        telemetry.log().add("Control mapping: [[" + controller.getClass().getSimpleName() + "]]");
        telemetry.log().add("Relativity     : [[" + (fieldCentric ? "Field" : "Robot") + " centric]]");
        telemetry.update();

        // Intake flipper servos are disabled by default
        waitForStart();
        ElapsedTime timeSinceMatchStart = new ElapsedTime();

        robot.markerDeployer.setPosition(AutoUtils.MARKER_DEPLOYER_RETRACTED);
        robot.parkingMarker.setPosition(AutoUtils.PARKING_MARKER_RETRACTED);
        loopTime.reset();
        robot.ledRiver.setMode(LEDRiver.Mode.PATTERN).apply();

        while (opModeIsActive()) {
            controller.update();

            // We will have a hierarchical set of colors
            // Here is the default color
            robot.ledRiver.setMode(LEDRiver.Mode.SOLID)
                    .setColor(Color.RED);

            // For macro move up and down, perform shortcuts
            if (controller.collectWithArm()) {
                arm.collect();
                robot.intake.collect();
                controller.setIntakeDir(-1);
                // Begin extend/retract operation after delay
                new java.util.Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                extender.goToCollect();
                            }
                        },
                        MS_DELAY_EXTEND
                );
            } else if (controller.depositWithArm()) {
                arm.deposit();
                controller.setIntakeDir(1);
                extender.goToMin();
                new java.util.Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                extender.goToMax();
                            }
                        },
                        MS_DELAY_EXTEND
                );
            } else {
                double desiredArmSpeed = controller.armSpeed();
                arm.setPower(desiredArmSpeed);
                telemetry.addData("Arm power %.2f", desiredArmSpeed);
                // Arm down means positive (this is because it is going
                // from the initial position (0))
                if (desiredArmSpeed > 0) {
                    telemetry.log().add("Collecting with conditional statement");
                    robot.intake.collect();
                }
                if (desiredArmSpeed <= POWER_USE_UP_DOWN_MACROS) {
                    timeMovingArmDown.reset();
                }
                if (desiredArmSpeed >= -POWER_USE_UP_DOWN_MACROS) {
                    timeMovingArmUp.reset();
                }
                if (controller.disableGP2Controls()) {
                    timeMovingArmDown.reset();
                    timeMovingArmUp.reset();
                }
            }


            if (timeMovingArmUp.milliseconds() > MS_USE_UP_MACROS) {
                extender.goToMax();
                controller.setIntakeDir(-1);
                armIsCollecting = false;
                robot.intake.deposit();
            }
            if (timeMovingArmDown.milliseconds() > MS_USE_DOWN_MACROS) {
                controller.setIntakeDir(-1);
                armIsCollecting = true;
            }


            if (controller.retakeControls()) {
                armIsCollecting = true;
            }

            if (armIsCollecting) {
                robot.ledRiver.setColor(Color.BLUE);
            }

            if (controller.openLatch()) {
                robot.blockTrapper.setPosition(BLOCK_TRAPPER_PERMISSIVE);
            } else {
                robot.blockTrapper.setPosition(BLOCK_TRAPPER_TRAPPING);
            }

            // Check to make sure
            int winchPower = controller.getHangDir();
            if (!robot.hangSwitch.getState()) {
                // If the switch is pressed (if it's not open)
                if (!controller.override()) {
                    winchPower = Math.max(winchPower, 0);
                }

                // Adjust the hang mech offset
                if (!winch.isBusy()) {
                    winchOffset = robot.winch.getTargetPosition();
                }
            }

            winch.setPower(winchPower);
            if (controller.flipOut()) {robot.intake.collect();}
            else if (controller.flipBack()) {
                robot.intake.deposit();
                controller.setIntakeDir(-1);
            } else if (controller.flipToMin()) {
                robot.intake.goToMin();
            }

            if (controller.quickReverse()) {
                robot.intake.setIntakeSpeed(Intake.MAX_INTAKE_SPEED * DIR_QUICK_REVERSE);
            } else {
                robot.intake.setIntakeSpeed(controller.getSpinSpeed());
            }


            WheelDriveVector speeds = new WheelDriveVector(controller.driveStickY(),
                    controller.driveStickX(), controller.turnSpeed());

            speeds.scale(controller.translateSpeedScale(), controller.turnSpeedScale());

            // Control linear slide extend retract and drive robot if necessary
            double slidePower = controller.getExtendSpeed();
            if (Math.abs(controller.armSpeed()) > 0.05 && robot.linearSlide.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                extender.setPower(slidePower + controller.armSpeed() * COUNTER_TURN_FACTOR);
            } else {
                extender.setPower(slidePower);
            }

            if (
                    ((extender.minExtend() && slidePower < 0) ||
                    (extender.maxExtend() && slidePower > 0)) &&
                            armIsCollecting) { // Don't move robot if we're not collecting
                speeds.forwardSpeed += slidePower * EXTEND_MAXED_DRIVE_POWER;
            }

            // Slew drive mapped to GP2 left/right
            if (armIsCollecting && !controller.disableGP2Controls()) { // Don't move robot if we're not collecting) {
                speeds.translateSpeed += controller.getSlewSpeed();
                speeds.turnSpeed += controller.getSlewSpeed() * SLEW_TURN_FACTOR;
                speeds.turnSpeed += controller.getGP2TurnSpeed();
            }

            if (controller.resetHeading()) {
                // If we reset, reset to crater centric controls
                if (hangOnCrater) {
                    headingOffset = robot.getHeading() + CRATER_HEADING_RESET;
                } else {
                    headingOffset = robot.getHeading() + DEPO_HEADING_RESET;
                }
            }
            // Control heading locking
            if (controller.lockTo45() || controller.lockTo225()) {
                // Pressing y overrides lock to 45
                double targetAngle = 0;
                if (controller.lockTo45() && hangOnCrater) {
                    targetAngle = CRATER_LANDER_DEPOSIT;
                } else if (controller.lockTo225() && hangOnCrater) {
                    targetAngle = CRATER_HANG;
                    endgameActivities = true;
                } else if (controller.lockTo45() && !hangOnCrater) {
                    targetAngle = DEPO_LANDER_DEPOSIT;
                } else {
                    targetAngle = DEPO_HANG;
                    endgameActivities = true;
                }

                double difference = robot.getSignedAngleDifference(targetAngle, robot.normAngle(robot.getHeading() - headingOffset));
                double turnSpeed = Math.max(-TURN_MAX_SPEED, Math.min(TURN_MAX_SPEED,
                        difference * TURN_CORRECT_FACTOR));
                turnSpeed = Math.copySign(Math.max(TURN_SPEED_CUTOFF, Math.abs(turnSpeed)), turnSpeed);
                speeds.turnSpeed = -turnSpeed;
            }

            if (controller.lockTo225() && !wasTurningTo255) {
                wasTurningTo255 = true;
                winch.setTargetPos(WINCH_MAX_POS + winchOffset);
            } else if (wasTurningTo255 && !controller.lockTo225()) {
                wasTurningTo255 = false;
            }

            robot.setMotorSpeeds(speeds.getDrivePowers());

            // Set LEDs
            if (!robot.onRawChassis) {
                double extendCurrent = leftHubEx.getMotorCurrentDraw(2);

                double servoCurrent = leftHubEx.getServoBusCurrentDraw() +
                        rightHubEx.getServoBusCurrentDraw();

                if (servoCurrent > SERVO_CURRENT_THRESHOLD) {
                    //robot.ledRiver.setColor(Color.WHITE);
                } else if (extendCurrent > EXTEND_CURRENT_THRESHOLD) {
                    robot.ledRiver.setColor(Color.GREEN);
                }

                telemetry.addData("Extend current", extendCurrent);
                telemetry.addData("Servo current", servoCurrent);
            }

            if (timing) {
                if (timeSinceMatchStart.seconds() > TIME_GAMEPLAY_DONE) {
                    robot.ledRiver.setMode(LEDRiver.Mode.PATTERN)
                            .setPattern(LEDRiver.Pattern.COLOR_WHEEL.builder());
                }
            }
            robot.ledRiver.apply();

            // Telemetry
            telemetry.addData("Time", Math.round(timeSinceMatchStart.seconds()));
            int pos = (robot.leftFlipper.getCurrentPosition() + robot.rightFlipper.getCurrentPosition()) / 2;
            telemetry.addData("Arm position", pos);
            telemetry.addData("Drive stick y", controller.driveStickY());
            telemetry.addData("Drive stick actual y", gamepad1.left_stick_y);
            telemetry.addData("Extender position", extender.getPosition());
            telemetry.addData("Mag switch", robot.slideSwitch.getState());

            telemetry.addData("Winch pos", robot.winch.getCurrentPosition());
            telemetry.addData("Loop time", loopTime.milliseconds());
            loopTime.reset();
            telemetry.update();
        }
    }
}
