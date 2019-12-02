package org.firstinspires.ftc.teamcode.Skystone;
import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MainTeleOpSky3", group="Linear Opmode")
public class MainTeleop extends LinearOpMode {
    Robot robot;
    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;
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
    public static double powerScaleFactor = 0.9;

    @Override
    public void runOpMode() {
        resetRobot();
        robot.initServos();
        waitForStart();
        robot.getOuttakeSpool().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Position2D position2D = new Position2D(robot);
//        position2D.startOdometry();
        while (opModeIsActive()) {
            telemetry.update();

            slowDriveLogic();
            driveLogic();

            intakeLogic();
            outtakeLogic();
            spoolLogic();

            foundationLogic();

            capStoneLogic();
            teamMarkerLogic();

//            telemetry.addLine("xPos: " + robot.getRobotPos().x);
//            telemetry.addLine("yPos: " + robot.getRobotPos().y);
//            telemetry.addLine("angle: " + robot.getAnglePos());
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

            robot.getBackStopper().setPosition(robot.BACK_STOPPER_DOWN);

            while(opModeIsActive() && SystemClock.elapsedRealtime() - startTime <250){
                driveLogic();
                slowDriveLogic();
                intakeLogic();
                foundationLogic();
            }

            robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED-0.12);

            robot.getOuttakeSpool().setPower(1);

            while(SystemClock.elapsedRealtime() - startTime < 1000 && opModeIsActive() && !gamepad1.x){
                driveLogic();
                slowDriveLogic();
                intakeLogic();
                foundationLogic();
            }

            robot.getOuttakeSpool().setPower(0);

            startTime = SystemClock.elapsedRealtime();
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_DUMP);

            while(SystemClock.elapsedRealtime()- startTime < 750){
                driveLogic();
                slowDriveLogic();
                intakeLogic();
                foundationLogic();
            }

            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_RETRACT);
        }

        robot.getOuttakeSpool().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void slowDriveLogic() {
        //toggle driving speed
        if (powerScaleFactor == 0.4) {
//            telemetry.addData("Driving Mode","Slow");
        } else {
//            telemetry.addData("Driving Mode","Normal");
        }
        if (gamepad1.left_bumper && !changedSlowDrive) {
            powerScaleFactor = (onSlowDrive) ? 0.9 : 0.4;
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

        if ((gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0) && gamepad2.right_trigger == 0) {
            robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED);
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);
            robot.getClamp().setPosition(0.32);
        }

        if(gamepad2.right_trigger != 0){
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_DOWN);
        }else if(gamepad2.left_trigger != 0){
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);
        }
    }
    private void outtakeLogic() {
        currentTime = SystemClock.elapsedRealtime();
        // Logic to control outtake; with a delay on the pivot so that the slides can extend before pivot rotation
        if (gamepad2.a) { // Clamp and Extend
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_RETRACT);
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);
            isExtend = true;
            isRetract = false;
            isClamp = false;
            is90 = false;
            outtakeExecutionTime = currentTime;
        } else if (gamepad2.b) { // Deposit and Reset
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_RETRACT);
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);
            robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_EXTENDED);
            robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED); // Reset intake pusher
            isRetract = true;
            isExtend = false;
            isClamp = false;
            is90 = false;
            outtakeExecutionTime = currentTime;
            robot.getClamp().setPosition(robot.CLAW_SERVO_RELEASED); // Release clamp
        }else if(gamepad2.x){
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_RETRACT);
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);
            isClamp = true;
            isExtend = false;
            isRetract = false;
            is90 = false;
            outtakeExecutionTime = currentTime;
            robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED);
        }else if(gamepad2.y){
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_RETRACT);
            robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);
            isExtend = true;
            isRetract = false;
            isClamp = false;
            is90 = true;
            outtakeExecutionTime = currentTime;
        }
        //extend
        if(currentTime - outtakeExecutionTime >= 250 && isExtend && !hasPushed){
            robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED); // Push block all the way to clamp
            hasPushed = true;
        }

        if (currentTime - outtakeExecutionTime >= 750 && isExtend) {
            robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED);
        }
        if (currentTime - outtakeExecutionTime >= 850 && isExtend) {
            robot.getClamp().setPosition(robot.CLAW_SERVO_CLAMPED);
        }
        if(currentTime-outtakeExecutionTime >= 950 && isExtend){
            robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_EXTENDED);
        }
        if(currentTime-outtakeExecutionTime >= 1650 && isExtend && !is90){
            robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_EXTENDED);
            isExtend = false;
            hasPushed = false;
        }

        //pivot 90
        if(currentTime-outtakeExecutionTime >= 1650 && isExtend && is90){
            robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_90);
            isExtend = false;
            is90 = false;
            hasPushed = false;
        }

        //retract
        if(currentTime-outtakeExecutionTime >= 450 && isRetract){
            robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_RETRACTED);
        }
        if(currentTime-outtakeExecutionTime >= 750 && isRetract){
            robot.getClamp().setPosition(robot.CLAW_SERVO_CLAMPED);
        }
        if(currentTime-outtakeExecutionTime >= 1500 && isRetract){
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

        robot.foundationMover(foundationToggle);
    }

    private void capStoneLogic() {
        if (gamepad1.x){
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_DUMP);
        } else if (gamepad1.y){
            robot.getMarkerServo().setPosition(robot.TEAM_MARKER_RETRACT);
        }
    }
}