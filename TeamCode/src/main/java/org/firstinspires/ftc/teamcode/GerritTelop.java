package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="GerritTelop")
public class GerritTelop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    int position;  // TODO This should be private and make this explicit it means nothing to me. robot_current_position?
    float now;     // TODO This should be private and make this explicit.  robot_last_known_position?
    boolean a;     // TODO I don't think you need this at all.

    public void leverArmStay() throws InterruptedException {
        //
        // Completely foreign to me?  Assuming not complete thought.
        //
        if (a = true) {
            a = false;
            sleep(10);
            now = robot.leverArm.getCurrentPosition();
        }
        if (a = false){
            a = true;

        }
    }
    public void leverArmStay2() throws InterruptedException {
        //
        // This is absolutely the right approach.
        //
        position = robot.leverArm.getCurrentPosition();
        telemetry.addData("position%.2d", position);
            if (position > now) {
                robot.leverArm.setPower(-.4);
            }
            if (position < now) {  // TODO else if..
                robot.leverArm.setPower(.4);
            }  // TODO ELSE set power 0

    }
    public void moveLeverArm(double distance) throws InterruptedException {
        telemetry.addData("Value %.2d", distance);
        position = robot.leverArm.getCurrentPosition() + 1;
        telemetry.addData("position%.2d", position);
        telemetry.update();

        if (distance > .1) {
            if (position >= robot.ARM_UP_DISTANCE) {
                robot.leverArm.setPower(0);
            }
            if (position >= 900) { // TODO - Shouldn't this be a elseif?
                robot.leverArm.setPower(-.27);
            }
            if (position <= robot.ARM_UP_DISTANCE) {  // TODO - Shouldn't this be a elseif?
                robot.leverArm.setPower(.35);
            }
        }
        if (distance < -.1) {
            if (position <= 100) {
                robot.leverArm.setPower(0);
            }
            if (position <= 900) {  // TODO - Shouldn't this be a elseif?
                robot.leverArm.setPower(.27);
            }
            if (position >= 100) {   // TODO - Shouldn't this be a elseif?
                robot.leverArm.setPower(-.35);
            }
        }
        // We need to record our last position
    }
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        a = true;
        waitForStart();
        int counter = 0;

        // Couple thoughts.  What is it we want to do..
        // When I let go of the stick I need it to freeze to the last known position.
        // I don't want to do anything more than that.  I wouldn't worry about the button A.
        // What I would suggest is that if left_stick_y == 0 and right_stick_y == 0 then stay!
        // Now the leverArmStay is simply going to take the currentPosition.  And we need to compare
        // that to our last recorded postion....
        // So...
        // You will need a top

        while (opModeIsActive()) {

            // I would remove this
            if (gamepad2.a) {
                leverArmStay();
            }
            if (a=false){
                leverArmStay2();
            }
            // I would remove this

            if (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) {
                moveLeverArm(-gamepad2.left_stick_y);
            }
            else {
                // This not goes to leverArmStay(getCurrentPosition) and remove setPower..
                robot.leverArm.setPower(0);
            }
        }
    }
}