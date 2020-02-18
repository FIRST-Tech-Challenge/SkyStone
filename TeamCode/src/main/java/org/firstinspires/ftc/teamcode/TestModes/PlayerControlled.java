package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

//by Lena and Simeon

@TeleOp(name = "Player_Controlled")

public class PlayerControlled extends OpMode {
    HardwareChassis robot;
    ColorTools colorTools;
    double smootingValue;
    double[] liftZeros = new double[2];

    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
        colorTools = new ColorTools();
        smootingValue = -0.5;

        setZeros();
    }

    @Override
    public void loop() {
        //smoothing value
        if (gamepad1.right_bumper) {
            smootingValue = smootingValue+0.05;
            while (gamepad1.right_bumper){}
        } else if (gamepad1.left_bumper) {
            smootingValue = smootingValue-0.05;
            while (gamepad1.left_bumper){}
        }

        //driving
        //else if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0) {
        double[] result = OmniWheel.calculate(
                5.0,
                38,
                24,
                GeneralTools.calculateControllerSmooting(-gamepad1.left_stick_y, smootingValue)*0.5,
                GeneralTools.calculateControllerSmooting(gamepad1.left_stick_x, smootingValue)*0.5,
                GeneralTools.calculateControllerSmooting(-gamepad1.right_trigger + gamepad1.left_trigger, smootingValue)); //gamepad1.right_stick_x*-0.2

        robot.motor_front_left.setPower(result[0]);
        robot.motor_front_right.setPower(result[1]);
        robot.motor_rear_left.setPower(result[2]);
        robot.motor_rear_right.setPower(result[3]);
        //}
    /*
        //turning
        else if (gamepad1.right_trigger > 0) {
            robot.motor_rear_right.setPower(-gamepad1.right_trigger*0.5);
            robot.motor_front_right.setPower(-gamepad1.right_trigger*0.5);
            robot.motor_rear_left.setPower(-gamepad1.right_trigger*0.5);
            robot.motor_front_left.setPower(-gamepad1.right_trigger*0.5);
        } else if (gamepad1.left_trigger > 0) {
            robot.motor_rear_right.setPower(gamepad1.left_trigger*0.5);
            robot.motor_front_right.setPower(gamepad1.left_trigger*0.5);
            robot.motor_rear_left.setPower(gamepad1.left_trigger*0.5);
            robot.motor_front_left.setPower(gamepad1.left_trigger*0.5);
        } else {
            robot.motor_rear_right.setPower(0);
            robot.motor_rear_left.setPower(0);
            robot.motor_front_right.setPower(0);
            robot.motor_front_left.setPower(0);
        }
        */

        //extender
        if (gamepad2.right_stick_y > 0 || gamepad2.right_stick_y < 0) {
            robot.motor_extender.setPower(-gamepad2.right_stick_y);
        } else {
            robot.motor_extender.setPower(0);
        }


        //* lift *//
        // upper limit
        if (gamepad2.left_stick_y < 0 && (
                (robot.motor_lift_left.getCurrentPosition() - liftZeros[0]) >= -7126 &&
                (robot.motor_lift_right.getCurrentPosition() - liftZeros[1]) <= 7126
        )) {
            robot.motor_lift_left.setPower(gamepad2.left_stick_y);
            robot.motor_lift_right.setPower(-gamepad2.left_stick_y);
        }
        // override/ignore upper limit
        else if (gamepad2.left_stick_y < 0 && gamepad2.b) {
            robot.motor_lift_left.setPower(gamepad2.left_stick_y);
            robot.motor_lift_right.setPower(-gamepad2.left_stick_y);
        }

        // lower limit
        else if (gamepad2.left_stick_y > 0 && (
                robot.motor_lift_left.getCurrentPosition() <= liftZeros[0] &&
                robot.motor_lift_right.getCurrentPosition() >= liftZeros[1]
        )) {
            robot.motor_lift_left.setPower(gamepad2.left_stick_y);
            robot.motor_lift_right.setPower(-gamepad2.left_stick_y);
        }
        // override/ignore lower limit
        else if (gamepad2.left_stick_y > 0 && gamepad2.b) {
            robot.motor_lift_left.setPower(gamepad2.left_stick_y);
            robot.motor_lift_right.setPower(-gamepad2.left_stick_y);
        }

        // set all motors 0 if nothing is pressed
        else {
            robot.motor_lift_left.setPower(0);
            robot.motor_lift_right.setPower(0);
        }
        // set new zero-point
        if (gamepad2.a) {
            setZeros();
        }

        if (gamepad2.dpad_up) {
            robot.servo_capstone.setPosition(0.4);
        }

        //servo clamp
        if(gamepad2.y) {
            GeneralTools.closeClamp(robot);
        } else if (gamepad2.x) {
            GeneralTools.openClamp(robot);
        }

        //servos foundation
        if (gamepad2.right_bumper) { //grab
            GeneralTools.grabFoundation(robot);
        } else if (gamepad2.left_bumper) { //release
            GeneralTools.releaseFoundation(robot);
        }


        //telemetry.addData("Smoothing Value: ", smootingValue);
        //telemetry.addLine();
        //telemetry.addData("Ärmchen R:", robot.servo_claw_right.getPosition());
        //telemetry.addData("Ärmchen L:", robot.servo_claw_left.getPosition());
        //telemetry.addLine();
        telemetry.addData("Touch Left: ", robot.touch_left.getState());
        telemetry.addData("Touch Right: ", robot.touch_right.getState());
        //telemetry.addLine();
        //telemetry.addData("H: ", colorTools.showHSV(robot.color_back)[0]);
        //telemetry.addData("S: ", colorTools.showHSV(robot.color_back)[1]);
        telemetry.addLine();
        telemetry.addData("LiftLpos: ", (robot.motor_lift_left.getCurrentPosition() - liftZeros[0]) / 712.6);
        telemetry.addData("LiftRpos: ", (robot.motor_lift_right.getCurrentPosition() - liftZeros[1]) / 712.6);
        telemetry.addData("ExtenderPos", robot.motor_extender.getCurrentPosition()/195.4);
        telemetry.update();
    }

    public void setZeros() {
        liftZeros[0] = robot.motor_lift_left.getCurrentPosition();
        liftZeros[1] = robot.motor_lift_right.getCurrentPosition();
    }
}


