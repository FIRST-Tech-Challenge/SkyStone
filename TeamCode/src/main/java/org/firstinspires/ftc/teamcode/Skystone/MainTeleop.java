package org.firstinspires.ftc.teamcode.Skystone;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.MotionAction;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(name = "MainTeleOpSky2", group = "Linear Opmode")
public class MainTeleop extends LinearOpMode {
    Robot robot;

    private Queue<MotionAction> outtakeActions = new LinkedList<>();

    private long currentTime;

    private boolean isTogglingG2A = false;
    private boolean isTogglingG2B = false;
    private boolean isTogglingG2X = false;
    private boolean isCapped = false;

    private double fLPower;
    private double fRPower;
    private double bLPower;
    private double bRPower;

    private double intakeLeftPower;
    private double intakeRightPower;

    private double spoolPosition;

    private double lastDropPosition = 0;
    private double spoolTargetPosition = 0;

    private boolean isMovingSpoolToPosition;
    private boolean isG2DPDPushed = false;

    private boolean onSlowDrive, changedSlowDrive = false;
    private boolean toggleMode = true;

    private boolean isRetract = false;
    private boolean isExtend = false;
    private boolean isClamp = false;
    private boolean is90 = false;
    private boolean foundationToggle = false;
    private boolean resetfoundation = false;

    private boolean isCapstone = false;
    private boolean toggleCap = false;

    private boolean isTogglingRB;
    private int indexPosition;
    private double spoolPower;

    private boolean isOverridingSlideFloor = false;

    private boolean motionExecuted;

    private static double powerScaleFactor = 0.9;

    private boolean isIntakeMode = false;

    private Position2D position2D;

    @Override
    public void runOpMode() {
        resetRobot();
        robot.initServos();

        waitForStart();

        position2D = new Position2D(robot);
        position2D.startOdometry();

        while (opModeIsActive()) {
            robotModeLogic();

            slowDriveLogic();
            driveLogic();

            foundationLogic();
            foundationMoveLogic();
            intakeLogic();

            if (!isIntakeMode) {
                telemetry.addLine("CURRENT ROBOT MODE: NORMAL");
                outtakeLogic();
                capstoneLogic();
                spoolLogic();
            } else {
                telemetry.addLine("CURRENT ROBOT MODE: INTAKE BOT");
            }

            if (robot.isDebug()) {
                telemetry.addLine("xPos: " + robot.getRobotPos().x);
                telemetry.addLine("yPos: " + robot.getRobotPos().y);
                telemetry.addLine("angle: " + Math.toDegrees(robot.getAnglePos()));
                telemetry.addLine("XPODLeft " + robot.getfLeft().getCurrentPosition());
                telemetry.addLine("XPODRight " + robot.getfRight().getCurrentPosition());
                telemetry.addLine("YPOD " + robot.getbLeft().getCurrentPosition());
                telemetry.addLine("target:" + spoolTargetPosition);
                telemetry.addLine("lastDropPosition:" + lastDropPosition);
                telemetry.addLine("index:" + indexPosition);

                telemetry.addLine("spoolPower: " + spoolPower);
                telemetry.addLine("isMovingSpoolToPosition: " + isMovingSpoolToPosition);
                telemetry.addLine("targetSpoolPosition: " + spoolTargetPosition);
                telemetry.addLine("Spool Position " + spoolPosition);
            }

            telemetry.update();
        }
    }

    private void spoolLogic() {
        spoolPosition = robot.getOuttakeSpool().getCurrentPosition();

        if (gamepad2.dpad_down && !isG2DPDPushed) {
            isMovingSpoolToPosition = true;
            if (lastDropPosition == 0) {
                spoolTargetPosition = 0;
            } else {
                if (isCapped) {
                    spoolTargetPosition = spoolTargetPosition - 200;
                } else {
                    spoolTargetPosition = spoolTargetPosition - 150;
                }
            }

            isG2DPDPushed = true;
        } else if (!gamepad2.dpad_down) {
            isG2DPDPushed = false;
        }

        if (gamepad2.right_bumper && !isTogglingRB) {
            isTogglingRB = true;
            double centerPosition = lastDropPosition;

            if (lastDropPosition == 0) {
                indexPosition = 0;
            } else {
                for (int i = 0; i < robot.spoolHeights.length; i++) {
                    if (robot.spoolHeights[i] > centerPosition) {
                        indexPosition = i + 1;
                        break;
                    }
                }
            }

            spoolTargetPosition = robot.spoolHeights[indexPosition];

            isMovingSpoolToPosition = true;
        } else if (!gamepad2.right_bumper) {
            isTogglingRB = false;
        }

        if (isMovingSpoolToPosition) {
            if (spoolTargetPosition < 0) {
                spoolTargetPosition = 0;
            }
            if (Math.abs(spoolPosition - spoolTargetPosition) < 50) {
                spoolPower = .125;
            } else if (spoolTargetPosition < spoolPosition) {
                spoolPower = (spoolTargetPosition - spoolPosition) / 1500;
            } else {
                spoolPower = (spoolTargetPosition - spoolPosition) / 200;
            }
        }

        //override mode
        if (gamepad2.left_stick_y != 0) {
            isMovingSpoolToPosition = false;
            isTogglingRB = false;
        }
        if (!isMovingSpoolToPosition) {
            spoolPower = -gamepad2.left_stick_y;
        }
        if (gamepad2.left_trigger != 0 && gamepad2.left_stick_y != 0) {
            spoolPower = -gamepad2.left_stick_y / 10000;
        } else if (gamepad2.left_trigger != 0) {
            isTogglingRB = false;

            spoolPower = .125;
        }

        if (spoolPosition >= 4200) {
            if (spoolPower == 0) {
                spoolPower = 0.125;
            }
            spoolPower = Math.min(spoolPower / 2, 0.15);
        }

        if (spoolPosition <= 0 && spoolPower < 0) {
            spoolPower = 0;
        }

        if (gamepad2.left_bumper) {
            spoolPower = -gamepad2.left_stick_y;
            isOverridingSlideFloor = true;
        } else if (isOverridingSlideFloor) {
            isOverridingSlideFloor = false;
            robot.getOuttakeSpool().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        robot.getOuttakeSpool().setPower(spoolPower);
        robot.getOuttakeSpool2().setPower(spoolPower);
    }

    private void resetRobot() {
        robot = new Robot(hardwareMap, telemetry, this);

        robot.setDrivetrainMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDrivetrainMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getFrontClamp().setDirection(Servo.Direction.FORWARD);

        robot.getOuttakeSpool().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getOuttakeSpool2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.getOuttakeSpool().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getOuttakeSpool().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getOuttakeSpool2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getIntakeLeft().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getIntakeRight().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.setBrakeModeDriveMotors();
    }

    //teleop methods
    private void driveLogic() {
        //tank drive
        fLPower = (-gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * powerScaleFactor;
        fRPower = (-gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * powerScaleFactor;
        bLPower = (-gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * powerScaleFactor;
        bRPower = (-gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * powerScaleFactor;
        if (gamepad1.right_trigger != 0) {
            fLPower = (gamepad1.right_trigger) * powerScaleFactor;
            fRPower = (-gamepad1.right_trigger) * powerScaleFactor;
            bLPower = (-gamepad1.right_trigger) * powerScaleFactor;
            bRPower = (gamepad1.right_trigger) * powerScaleFactor;
        } else if (gamepad1.left_trigger != 0) {
            fLPower = (-gamepad1.left_trigger) * powerScaleFactor;
            fRPower = (gamepad1.left_trigger) * powerScaleFactor;
            bLPower = (gamepad1.left_trigger) * powerScaleFactor;
            bRPower = (-gamepad1.left_trigger) * powerScaleFactor;
        }
        //Straight D-Pad move
        if (gamepad1.dpad_up) {
            fLPower = (gamepad1.left_stick_y) + powerScaleFactor;
            bLPower = (gamepad1.left_stick_y) + powerScaleFactor;
            fRPower = (gamepad1.right_stick_y + powerScaleFactor);
            bRPower = (gamepad1.right_stick_y + powerScaleFactor);
        } else if (gamepad1.dpad_down) {
            fLPower = (gamepad1.left_stick_y) - powerScaleFactor;
            bLPower = (gamepad1.left_stick_y) - powerScaleFactor;
            fRPower = (gamepad1.right_stick_y - powerScaleFactor);
            bRPower = (gamepad1.right_stick_y) - powerScaleFactor;
        } else if (gamepad1.dpad_right) {
            fLPower = (gamepad1.right_stick_y) + powerScaleFactor;
            bLPower = (gamepad1.right_stick_y) + powerScaleFactor;
            fRPower = (gamepad1.left_stick_y) - powerScaleFactor;
            bRPower = (gamepad1.left_stick_y) - powerScaleFactor;
        } else if (gamepad1.dpad_left) {
            fRPower = (gamepad1.right_stick_y) + powerScaleFactor;
            bRPower = (gamepad1.right_stick_y) + powerScaleFactor;
            fLPower = (gamepad1.left_stick_y) - powerScaleFactor;
            bLPower = (gamepad1.left_stick_y) - powerScaleFactor;
        }

        robot.allWheelDrive(fLPower, fRPower, bLPower, bRPower);
    }

    private void capstoneLogic() {
        if (gamepad2.right_trigger != 0 && !toggleCap) {
            robot.getIntakeLeft().setPower(1);
            robot.getIntakeRight().setPower(1);

            isCapped = true;
            toggleCap = true;
            robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED);
            long startTime = SystemClock.elapsedRealtime();

            while (SystemClock.elapsedRealtime() - startTime <= 1000) {
                robotModeLogic();

                slowDriveLogic();
                driveLogic();

                foundationLogic();

                spoolLogic();
                intakeLogic();
            }
            robot.getIntakeLeft().setPower(0);
            robot.getIntakeRight().setPower(0);

            startTime = SystemClock.elapsedRealtime();
            robot.getFrontClamp().setPosition(robot.FRONTCLAMP_ACTIVATECAPSTONE);

            while (SystemClock.elapsedRealtime() - startTime <= 500) {
                robotModeLogic();

                slowDriveLogic();
                driveLogic();

                foundationLogic();

                spoolLogic();
                intakeLogic();
            }
            startTime = SystemClock.elapsedRealtime();
            robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED);

            while (SystemClock.elapsedRealtime() - startTime <= 200) {
                robotModeLogic();

                slowDriveLogic();
                driveLogic();

                foundationLogic();

                spoolLogic();
                intakeLogic();
            }

            robot.getFrontClamp().setPosition(robot.FRONTCLAMP_CLAMPED);
            robot.getBackClamp().setPosition(robot.BACKCLAMP_CLAMPED);

        } else if (!(gamepad2.right_trigger != 0)) {
            toggleCap = false;
        }
    }

    private void slowDriveLogic() {
        //toggle driving speed
        if (gamepad1.left_bumper && !changedSlowDrive) {
            powerScaleFactor = (onSlowDrive) ? 0.9 : 0.65;
            onSlowDrive = !onSlowDrive;
            changedSlowDrive = true;
        } else if (!gamepad1.left_bumper) {
            changedSlowDrive = false;
        }
    }

    private boolean stoneInIntake = false;

    private void intakeLogic() {
        intakeLeftPower = gamepad2.right_stick_y;
        intakeRightPower = gamepad2.right_stick_y;

        if (isIntakeMode) {
            robot.setAutoStopIntake(true);
            robot.getBackClamp().setPosition(robot.BACKCLAMP_CLAMPED);
            robot.getFrontClamp().setPosition(robot.FRONTCLAMP_CLAMPED);
            robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED);

            if (Double.isNaN(robot.getIntakeStoneDistance().getDistance(DistanceUnit.CM))) {
                stoneInIntake = false;
            }else if (robot.getIntakeStoneDistance().getDistance(DistanceUnit.CM) < 40) {
                stoneInIntake = true;
            }

            if (stoneInIntake) {
                if (intakeLeftPower > 0) {
                    intakeLeftPower = 0;
                }
                if (intakeRightPower > 0) {
                    intakeRightPower = 0;
                }
            }
        } else if(intakeRightPower != 0 && intakeLeftPower != 0) {
            robot.setAutoStopIntake(false);
            robot.getBackClamp().setPosition(robot.BACKCLAMP_CLAMPED);
            robot.getFrontClamp().setPosition(robot.FRONTCLAMP_RELEASED);
        }

        robot.getIntakeLeft().setPower(intakeLeftPower);
        robot.getIntakeRight().setPower(intakeRightPower);
    }

    private void outtakeLogic() {
        currentTime = SystemClock.elapsedRealtime();
        // Logic to control outtake; with a delay on the pivot so that the slides can extend before pivot rotation
        if (gamepad2.a && !isTogglingG2A) { // Clamp and Extend
            isTogglingG2A = true;

            outtakeActions.clear();

            outtakeActions.add(new MotionAction(robot.getOuttakeExtender(), robot.OUTTAKE_SLIDE_EXTENDED, currentTime + robot.DELAY_SLIDE_ON_EXTEND, robot));
        } else if (!gamepad2.a) {
            isTogglingG2A = false;
        }

        if (gamepad2.b && !isTogglingG2B) { // Deposit and Reset
            lastDropPosition = robot.getOuttakeSpool().getCurrentPosition();
            isMovingSpoolToPosition = false;

            isTogglingG2B = true;

            outtakeActions.clear();

            outtakeActions.add(new MotionAction(robot.getIntakePusher(), robot.PUSHER_RETRACTED, currentTime + robot.DELAY_PUSHER_ON_RETRACT, robot));
            outtakeActions.add(new MotionAction(robot.getBackClamp(), robot.BACKCLAMP_RELEASED, currentTime + robot.DELAY_RELEASE_CLAMP_ON_RETRACT, robot));
            outtakeActions.add(new MotionAction(robot.getFrontClamp(), robot.FRONTCLAMP_RELEASED, currentTime + robot.DELAY_RELEASE_CLAMP_ON_RETRACT, robot));
            outtakeActions.add(new MotionAction(robot.getOuttakeExtender(), robot.OUTTAKE_SLIDE_RETRACTED, currentTime + robot.DELAY_SLIDE_ON_RETRACT, robot));
        } else if (!gamepad2.b) {
            isTogglingG2B = false;
        }

        if (gamepad2.x && !isTogglingG2X) {
            isTogglingG2X = true;

            outtakeActions.clear();

            outtakeActions.add(new MotionAction(robot.getIntakePusher(), robot.PUSHER_PUSHED, currentTime + robot.DELAY_PUSHER_ON_CLAMP, robot));
            outtakeActions.add(new MotionAction(robot.getIntakePusher(), robot.PUSHER_RETRACTED, currentTime + robot.DELAY_RETRACT_PUSHER_ON_CLAMP, robot));
            outtakeActions.add(new MotionAction(robot.getFrontClamp(), robot.FRONTCLAMP_CLAMPED, currentTime + robot.DELAY_CLAMP_ON_CLAMP, robot));
            outtakeActions.add(new MotionAction(robot.getBackClamp(), robot.BACKCLAMP_CLAMPED, currentTime + robot.DELAY_CLAMP_ON_CLAMP, robot));
        } else if (!gamepad2.x) {
            isTogglingG2X = false;
        }

        motionExecuted = true;
        while (motionExecuted) {
            currentTime = SystemClock.elapsedRealtime();
            MotionAction currentMotion = outtakeActions.peek();

            if (currentMotion != null) {
                if (currentMotion.getDelayStartTime() <= currentTime) {
                    currentMotion.executeMotion();
                    outtakeActions.remove();
                } else {
                    motionExecuted = false;
                }
            } else {
                motionExecuted = false;
            }
        }
    }

    private long foundationRaiseTime = 0;

    private void foundationLogic() {
        if (gamepad1.right_bumper) {
            if (foundationToggle && !resetfoundation) {
                foundationToggle = false;
                foundationRaiseTime = SystemClock.elapsedRealtime();
            } else if (!foundationToggle && !resetfoundation) {
                foundationToggle = true;
            }
            resetfoundation = true;
        } else {
            resetfoundation = false;
        }

        if (!foundationToggle && (SystemClock.elapsedRealtime() - 750 > foundationRaiseTime)) {
            robot.getLeftFoundation().getController().pwmDisable();
        } else {
            robot.getLeftFoundation().getController().pwmEnable();
        }

        robot.foundationMovers(foundationToggle);
    }

    private void foundationMoveLogic() {
        if (gamepad1.a) {
            robot.foundationMovers(true);

            robot.setDrivetrainMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setDrivetrainMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            position2D.o.resetOdometry();

            robot.moveToPoint(24.5, 0, 1, 0.9, 0);
        }
    }

    private void robotModeLogic() {
        if (gamepad1.y && gamepad1.b && toggleMode) {
            isIntakeMode = !isIntakeMode;

            if (!isIntakeMode) {
                stoneInIntake = false;
            }

            toggleMode = false;
        } else if (!toggleMode && !(gamepad1.y && gamepad1.b)) {
            toggleMode = true;
        }
    }
}