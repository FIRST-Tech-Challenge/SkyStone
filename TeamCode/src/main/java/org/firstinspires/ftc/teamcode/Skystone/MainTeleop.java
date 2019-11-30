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
    double intakeLeftPower;
    double intakeRightPower;
    char outtakeButton;
    long outtakeExecutionTime;
    long currentTime;
    double outtakePivotExecutePosition;
    double outtakePivotWaitTime;
    double outtakeExtenderExecutePosition;
    double outtakeExtenderWaitTime;
    double outtakeClampExecutePosition;
    double outtakeClampWaitTime;
    boolean onSlowDrive, changedSlowDrive = false;
    boolean isRetract = false;
    boolean isExtend = false;
    boolean isClamp = false;
    boolean is90 = false;
    boolean outtakeExtended = false;
    boolean foundationToggle = false;
    boolean resetfoundation = false;
    public static double powerScaleFactor = 0.9;
    @Override
    public void runOpMode() {
        resetRobot();
        initServos();
        waitForStart();
        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();
        while (opModeIsActive()) {
            telemetry.update();
            slowDriveLogic();
            driveLogic();
//            outtakeLogic();
            intakeLogic();
            spoolLogic();
            outtakeLogic();
            foundationLogic();
            capStoneLogic();

            telemetry.addLine("xPos: " + robot.getRobotPos().x);
            telemetry.addLine("yPos: " + robot.getRobotPos().y);
            telemetry.addLine("angle: " + robot.getAnglePos());
        }
    }
    private void spoolLogic(){
        if (gamepad2.dpad_up ) {
            robot.getOuttakeSpool().setPower(1);
        }else if(gamepad2.dpad_down  ){
            robot.getOuttakeSpool().setPower(-1);
        }else{
            robot.getOuttakeSpool().setPower(0);
        }
//        telemetry.addLine("Spool Position " + robot.getOuttakeSpool().getCurrentPosition());
        telemetry.update();
    }
    private void resetRobot() {
        robot = new Robot(hardwareMap, telemetry, this);
        robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getClampPivot().setDirection(Servo.Direction.FORWARD);
        robot.getOuttakeSpool().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getIntakeLeft().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getIntakeRight().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.outtakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.outtakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void initServos() {
        robot.foundationMover(false);

        isRetract = true;
        outtakeExecutionTime = SystemClock.elapsedRealtime();

        robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED);
        robot.getClamp().setPosition(robot.CLAW_SERVO_RELEASED);

        while (isRetract) {
            currentTime = SystemClock.elapsedRealtime();
            if (currentTime - outtakeExecutionTime >= 250 && isRetract) {
                robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_RETRACTED);
            }
            if (currentTime - outtakeExecutionTime >= 950 && isRetract) {
                robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_RETRACTED);
                isRetract = false;
            }
        }
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
    private void slowDriveLogic() {
        //toggle driving speed
        if (powerScaleFactor == 0.3) {
//            telemetry.addData("Driving Mode","Slow");
        } else {
//            telemetry.addData("Driving Mode","Normal");
        }
        if (gamepad1.left_bumper && !changedSlowDrive) {
            powerScaleFactor = (onSlowDrive) ? 0.9 : 0.3;
            onSlowDrive = !onSlowDrive;
            changedSlowDrive = true;
        } else if (!gamepad1.left_bumper) {
            changedSlowDrive = false;
        }
    }
    private void intakeLogic() {
        if (!outtakeExtended) {
            robot.getIntakeLeft().setPower(gamepad2.left_stick_y);
            robot.getIntakeRight().setPower(gamepad2.right_stick_y);
        } else {
            robot.getIntakeLeft().setPower(0);
            robot.getIntakeRight().setPower(0);
        }

        if (gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0) {
            robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED);
            robot.getClamp().setPosition(0.32);
        }
    }
    private void outtakeLogic() {
        currentTime = SystemClock.elapsedRealtime();
        // Logic to control outtake; with a delay on the pivot so that the slides can extend before pivot rotation
        if (gamepad2.a) { // Clamp and Extend
            robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED); // Push block all the way to clamp
            isExtend = true;
            isRetract = false;
            isClamp = false;
            is90 = false;
            outtakeExecutionTime = currentTime;
        } else if (gamepad2.b) { // Deposit and Reset
            robot.enableOuttakeExtender();
            robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_EXTENDED);
            robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED); // Reset intake pusher
            isRetract = true;
            isExtend = false;
            isClamp = false;
            is90 = false;
            outtakeExecutionTime = currentTime;
            robot.getClamp().setPosition(robot.CLAW_SERVO_RELEASED); // Release clamp
        }else if(gamepad2.x){
            isClamp = true;
            isExtend = false;
            isRetract = false;
            is90 = false;
            outtakeExecutionTime = currentTime;
            robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED);
        }else if(gamepad2.y){
            robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED); // Push block all the way to clamp
            isExtend = true;
            isRetract = false;
            isClamp = false;
            is90 = true;
            outtakeExecutionTime = currentTime;
        }
        //extend
        if (currentTime - outtakeExecutionTime >= 250 && isExtend) {
            robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED);
        }
        if (currentTime - outtakeExecutionTime >= 350 && isExtend) {
            robot.getClamp().setPosition(robot.CLAW_SERVO_CLAMPED);
        }
        if(currentTime-outtakeExecutionTime >= 450 && isExtend){
            robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_EXTENDED);
        }
        if(currentTime-outtakeExecutionTime >= 1150 && isExtend && !is90){
            robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_EXTENDED);
            isExtend = false;
            robot.disableOuttakeExtender();
        }

        //pivot 90
        if(currentTime-outtakeExecutionTime >= 1150 && isExtend && is90){
            robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_90);
            isExtend = false;
            is90 = false;
            robot.disableOuttakeExtender();
        }

        //retract
        if(currentTime-outtakeExecutionTime >= 450 && isRetract){
            robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_RETRACTED);
        }
        if(currentTime-outtakeExecutionTime >= 750 && isRetract){
            robot.getClamp().setPosition(robot.CLAW_SERVO_CLAMPED);
        }
        if(currentTime-outtakeExecutionTime >= 1250 && isRetract){
            robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_RETRACTED);
            isRetract = false;
        }

        //clamp only
        if(currentTime-outtakeExecutionTime >= 300 && isClamp){
            robot.getClamp().setPosition(robot.CLAW_SERVO_CLAMPED);
            isClamp = false;
            robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED);
        }
    }

    private void foundationLogic() {
        if (gamepad1.left_bumper) {
            if (foundationToggle && !resetfoundation) {
                foundationToggle = false;
            } else if (!foundationToggle && !resetfoundation){
                foundationToggle = true;
            }
            resetfoundation = true;
        } else {
            resetfoundation = false;
        }

        robot.foundationMover(foundationToggle);
    }

    private void capStoneLogic() {
        if (gamepad1.a){
            robot.getMarkerServo().setPosition(1);
        } else if (gamepad1.b){
            robot.getMarkerServo().setPosition(0.3);
        }
    }
}