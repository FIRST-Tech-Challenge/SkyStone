package org.firstinspires.ftc.teamcode.Skystone;
import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;


@TeleOp(name="MainTeleOpSky", group="Linear Opmode")
public class MainTeleop extends LinearOpMode {
    Robot robot;

    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;

    double spoolPower;

    long outtakeExecutionTime;
    long currentTime;

    boolean onSlowDrive, changedSlowDrive = false;

    boolean isRetract = false;
    boolean isExtend = false;
    boolean isClamp = false;
    boolean is90 = false;
    boolean outtakeExtended = false;
    boolean foundationToggle = false;
    boolean resetfoundation = false;
    boolean hasPushed = false;
    boolean isRetractingSlides = false;
    boolean hasRetracted = false;

    public static double powerScaleFactor = 0.9;

    boolean isIntakeMode = false;


    // Booleans for debugging
    boolean isTelemetryPosition = true;

    @Override
    public void runOpMode() {
        resetRobot();
        robot.initServos();
        waitForStart();

        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();

        while (opModeIsActive()) {
            telemetry.update();

            robotModeLogic();

            slowDriveLogic();
            driveLogic();

            intakeLogic();

            foundationLogic();

            if (!isIntakeMode) {
                outtakeLogic();
                capStoneLogic();
                teamMarkerLogic();
                spoolLogic();
            }

            if (isTelemetryPosition) {
                telemetry.addLine("xPos: " + robot.getRobotPos().x);
                telemetry.addLine("yPos: " + robot.getRobotPos().y);
                telemetry.addLine("angle: " + Math.toDegrees(MathFunctions.angleWrap(robot.getAnglePos())));
                telemetry.addLine("XPODLeft " + robot.getfLeft().getCurrentPosition());
                telemetry.addLine("XPODRight " + robot.getfRight().getCurrentPosition());
                telemetry.addLine("YPOD " + robot.getbLeft().getCurrentPosition());
            }

            if (isIntakeMode) {
                telemetry.addLine("CURRENT ROBOT MODE: INTAKE BOT");
            } else {
                telemetry.addLine("CURRENT ROBOT MODE: NORMAL");
            }

            telemetry.addLine("outtake spool: " + robot.getOuttakeSpool().getCurrentPosition());
        }
    }

    boolean isToggleRetractingSlides = false;
    private void spoolLogic(){
        if (gamepad2.dpad_up) {
            isRetractingSlides = false;
            spoolPower = 1;
        }else if(gamepad2.dpad_down){
            isRetractingSlides = false;
            spoolPower = -1;
        }else if(!isRetractingSlides){
            spoolPower = 0;
        }

        if(gamepad2.right_bumper && !isToggleRetractingSlides){
            if (isRetractingSlides) {
                isRetractingSlides = false;
            } else {
                isRetractingSlides = true;
            }
            isToggleRetractingSlides = true;
        } else if (!gamepad2.right_bumper && isToggleRetractingSlides) {
            isToggleRetractingSlides = false;
        }

        if(isRetractingSlides){
            spoolPower = -1;
        }

        if(isRetractingSlides && (robot.getOuttakeSpool().getCurrentPosition()>=0)){
            spoolPower = 0;
            isRetractingSlides = false;
        }

        robot.getOuttakeSpool().setPower(spoolPower);
        robot.getOuttakeSpool2().setPower(spoolPower);
//        telemetry.addLine("Spool Position " + robot.getOuttakeSpool().getCurrentPosition());
    }

    private void resetRobot() {
        robot = new Robot(hardwareMap, telemetry, this);

        robot.setDrivetrainMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDrivetrainMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getClampPivot().setDirection(Servo.Direction.FORWARD);

        robot.getOuttakeSpool().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getOuttakeSpool2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getOuttakeSpool().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getOuttakeSpool().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getOuttakeSpool2().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.getIntakeLeft().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getIntakeRight().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //teleop methods
    private void driveLogic() {
        // TODO: change all of this stuff to x, y, and turn movements
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

    private void teamMarkerLogic(){
        if(gamepad2.left_bumper){
            long startTime = SystemClock.elapsedRealtime();
            robot.getClamp().setPosition(robot.CLAMP_SERVO_INTAKEPOSITION);
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_DOWN);

            while(opModeIsActive() && SystemClock.elapsedRealtime() - startTime <250){
                driveLogic();
                slowDriveLogic();
                intakeLogic();
                foundationLogic();
            }

            robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED-0.12);

            robot.getOuttakeSpool().setPower(1);
            robot.getOuttakeSpool2().setPower(1);

            while(SystemClock.elapsedRealtime() - startTime < 1000 && opModeIsActive() && !gamepad1.x){
                driveLogic();
                slowDriveLogic();
                intakeLogic();
                foundationLogic();
            }

            robot.getOuttakeSpool().setPower(0);
            robot.getOuttakeSpool2().setPower(0);

            startTime = SystemClock.elapsedRealtime();
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_DUMP);

            while(SystemClock.elapsedRealtime() - startTime < 750){
                driveLogic();
                slowDriveLogic();
                intakeLogic();
                foundationLogic();
            }

            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_RETRACT);

            while(SystemClock.elapsedRealtime() - startTime < 1000){
                driveLogic();
                slowDriveLogic();
                intakeLogic();
                foundationLogic();
            }

            while(robot.getOuttakeSpool().getCurrentPosition() <= 0){
                driveLogic();
                slowDriveLogic();
                intakeLogic();
                foundationLogic();
                robot.getOuttakeSpool().setPower(-1);
                robot.getOuttakeSpool2().setPower(-1);
            }

            startTime = SystemClock.elapsedRealtime();

            robot.getOuttakeSpool().setPower(0);
            robot.getOuttakeSpool2().setPower(0);

            robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);

            while(SystemClock.elapsedRealtime() - startTime < 250){
                driveLogic();
                slowDriveLogic();
                intakeLogic();
                foundationLogic();
            }

            robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED);

            while(SystemClock.elapsedRealtime()- startTime < 500){
                driveLogic();
                slowDriveLogic();
                intakeLogic();
                foundationLogic();
            }

            robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED);
            robot.getClamp().setPosition(robot.CLAMP_SERVO_CLAMPED);
        }
    }

    private void slowDriveLogic() {
        //toggle driving speed
        if (gamepad1.left_bumper && !changedSlowDrive) {
            powerScaleFactor = (onSlowDrive) ? 0.9 : 0.4;
            onSlowDrive = !onSlowDrive;
            changedSlowDrive = true;
        } else if (!gamepad1.left_bumper) {
            changedSlowDrive = false;
        }

//        if (powerScaleFactor == 0.4) {
////            telemetry.addData("Driving Mode","Slow");
//        } else {
////            telemetry.addData("Driving Mode","Normal");
//        }
    }

    private void intakeLogic() {
        if (!outtakeExtended) {
            robot.getIntakeLeft().setPower(gamepad2.left_stick_y);
            robot.getIntakeRight().setPower(gamepad2.right_stick_y);
        } else {
            robot.getIntakeLeft().setPower(0);
            robot.getIntakeRight().setPower(0);
        }

        if(robot.getClamp().getPosition() != robot.CLAMP_SERVO_INTAKEPOSITION && !isIntakeMode){
            robot.getIntakeLeft().setPower(0);
            robot.getIntakeRight().setPower(0);
        }

        if ((gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0) && gamepad2.right_trigger == 0 && !isExtend && !isClamp && !is90 && !isRetract) {
            if (isIntakeMode) {
                robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED);
                robot.getClamp().setPosition(robot.CLAMP_SERVO_CLAMPED);
            } else {
                robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED);
                robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);
                robot.getClamp().setPosition(robot.CLAMP_SERVO_INTAKEPOSITION);
            }
        }

        if(gamepad2.right_trigger != 0){
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_DOWN);
        }else if(gamepad2.left_trigger != 0){
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);
        }
    }

    double xDump;
    double yDump;
    private void outtakeLogic() {
        currentTime = SystemClock.elapsedRealtime();
        // Logic to control outtake; with a delay on the pivot so that the slides can extend before pivot rotation
        if (gamepad2.a) { // Clamp and Extend
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_RETRACT);
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);
            isExtend = true;
            isRetract = false;
            is90 = false;
            outtakeExecutionTime = currentTime;
            robot.getClamp().setPosition(robot.CLAMP_SERVO_CLAMPED);
            robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_EXTENDED);
        } else if (gamepad2.b) { // Deposit and Reset
            xDump = robot.getRobotPos().x;
            yDump = robot.getRobotPos().y;
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_RETRACT);
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);

//            robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED); // Reset intake pusher
            isRetract = true;
            hasRetracted = false;
            isExtend = false;
            isClamp = false;
            is90 = false;
            outtakeExecutionTime = currentTime;
            robot.getClamp().setPosition(robot.CLAMP_SERVO_RELEASED); // Release clamp
        }else if(gamepad2.x){
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_RETRACT);
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);
            isClamp = true;
            isExtend = false;
            isRetract = false;
            is90 = false;
            outtakeExecutionTime = currentTime;
            robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED);
            hasPushed = true;
        }else if(gamepad2.y){
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_RETRACT);
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);
            isExtend = true;
            isRetract = false;
            isClamp = false;
            is90 = true;
            outtakeExecutionTime = currentTime;
        }

        if(currentTime-outtakeExecutionTime >= 900 && isExtend){
            robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_EXTENDED);
        }

        if(currentTime-outtakeExecutionTime >= 1200 && isExtend && !is90){
            robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_PARTIAL_EXTEND);
            isExtend = false;
            hasPushed = false;
        }

        //pivot 90
        if(currentTime-outtakeExecutionTime >= 2000 && isExtend && is90){
            robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_90);
            isExtend = false;
            is90 = false;
            hasPushed = false;
        }

        //retract
        if(isRetract && (Math.hypot(robot.getRobotPos().x-xDump, robot.getRobotPos().y-yDump) > 5 || gamepad1.b) && !hasRetracted){
            robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_EXTENDED);
            outtakeExecutionTime = SystemClock.elapsedRealtime();
            hasRetracted = true;
        }

        if(currentTime-outtakeExecutionTime >= 450 && isRetract && hasRetracted){
            robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_RETRACTED);
        }
        if(currentTime-outtakeExecutionTime >= 750 && isRetract && hasRetracted){
            robot.getClamp().setPosition(robot.CLAMP_SERVO_CLAMPED);
        }
        if(currentTime-outtakeExecutionTime >= 1200 && isRetract && hasRetracted){
            robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_RETRACTED);
            isRetract = false;
        }

        //clamp only
        if(currentTime-outtakeExecutionTime >= 300 && isClamp){
            robot.getClamp().setPosition(robot.CLAMP_SERVO_CLAMPED);
            isClamp = false;
            robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED);
        }
    }

    private void foundationLogic() {
        if (gamepad1.right_bumper) {
            if (foundationToggle && !resetfoundation) {
                foundationToggle = false;
            } else if (!foundationToggle && !resetfoundation){
                foundationToggle = true;
            }
            resetfoundation = true;
        } else {
            resetfoundation = false;
        }

        robot.foundationMovers(foundationToggle);
    }

    private void capStoneLogic() {
        if (gamepad1.x){
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_DUMP);
        } else if (gamepad1.y){
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_RETRACT);
        }
    }

    private boolean toggleMode = true;
    private void robotModeLogic() {
        if (gamepad1.a && gamepad1.y && toggleMode) {
            if (isIntakeMode) {
                isIntakeMode = false;
            } else {
                isIntakeMode = true;
            }
            toggleMode = false;
        } else if (!toggleMode && !(gamepad1.a || gamepad1.y)) {
            toggleMode = true;
        }
    }
}