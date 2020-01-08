package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

//by Lena and Simeon

@TeleOp(name = "FullTest")

public class FullTest extends OpMode {
    HardwareChassis robot;
    ColorTools colorTools;
    double smootingValue;
    double[] liftZeros = new double[2];

    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
        colorTools = new ColorTools();
        smootingValue = -0.5;

        liftZeros[0] = robot.motor_lift_left.getCurrentPosition();
        liftZeros[0] = robot.motor_lift_right.getCurrentPosition();
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
                GeneralTools.calculateControllerSmooting(-gamepad1.left_trigger + gamepad1.right_trigger, smootingValue)); //gamepad1.right_stick_x*-0.2

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

        //clamp
        if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
            robot.motor_extender.setPower(-gamepad2.right_trigger + gamepad2.left_trigger);
        } else {
            robot.motor_extender.setPower(0);
        }
        //lift

        if (-gamepad2.left_stick_y > 0) {
            robot.motor_lift_left.setPower(0.1 * gamepad2.left_stick_y);
            robot.motor_lift_right.setPower(-0.1 * gamepad2.left_stick_y);
        } else if (-gamepad2.left_stick_y < 0 && (robot.motor_lift_left.getCurrentPosition() <= liftZeros[0] && robot.motor_lift_right.getCurrentPosition() >= liftZeros[1])) {
            robot.motor_lift_left.setPower(0.1 * gamepad2.left_stick_y);
            robot.motor_lift_right.setPower(-0.1 * gamepad2.left_stick_y);
        }
        //set all motors 0 if nothing is pressed
        else {
            robot.motor_lift_left.setPower(0);
            robot.motor_lift_right.setPower(0);
        }


        //servo clamp
        if(gamepad2.y) {
            robot.servo_grab.setPosition(1); //close
        } else if (gamepad2.x) {
            robot.servo_grab.setPosition(0.6); //open
        }

        //servos foundation
        if (gamepad2.right_bumper) {
            robot.servo_claw_right.setPosition(0.5);
            robot.servo_claw_left.setPosition(0.5);
        } else if (gamepad2.left_bumper) {

            robot.servo_claw_right.setPosition(0.9);
            robot.servo_claw_left.setPosition(0.1);
        }


        telemetry.addData("Smoothing Value: ", smootingValue);
        telemetry.addLine();
        telemetry.addData("Ärmchen R:", robot.servo_claw_right.getPosition());
        telemetry.addData("Ärmchen L:", robot.servo_claw_left.getPosition());
        telemetry.addLine();
        telemetry.addData("Touch Left: ", robot.touch_left.getState());
        telemetry.addData("Touch Right: ", robot.touch_right.getState());
        telemetry.addLine();
        telemetry.addData("H: ", colorTools.showHSV(robot.color_front)[0]);
        telemetry.addData("S: ", colorTools.showHSV(robot.color_front)[1]);
        telemetry.addData("V: ", colorTools.showHSV(robot.color_front)[2]);
        telemetry.update();

    }
}


