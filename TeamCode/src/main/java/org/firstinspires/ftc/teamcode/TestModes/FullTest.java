package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

@TeleOp(name = "FullTest")

public class FullTest extends OpMode {
    HardwareChassis robot;
    ColorTools colorTools;
    double smootingValue;

    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
        colorTools = new ColorTools();
        smootingValue = -0.5;
    }

    @Override
    public void loop() {
        double[] result = OmniWheel.calculate(
                5.0,
                38,
                24,
                GeneralTools.calculateControllerSmooting(-gamepad1.left_stick_y, smootingValue)*0.5,
                GeneralTools.calculateControllerSmooting(gamepad1.left_stick_x, smootingValue)*0.5,
                GeneralTools.calculateControllerSmooting(gamepad1.right_stick_x, smootingValue)*-0.02);

        /*
        double[] result = OmniWheel.calculate(
                5.0,
                38,
                24,
                -gamepad1.left_stick_y*0.5,
                gamepad1.left_stick_x*0.5,
                gamepad1.right_stick_x*0.05);
         */

        if (gamepad1.right_bumper) {
            smootingValue = smootingValue+0.05;
            while (gamepad1.right_bumper){}
        }
        if (gamepad1.left_bumper) {
            smootingValue = smootingValue-0.05;
            while (gamepad1.left_bumper){}
        }


        robot.motor_front_left.setPower(result[0]);
        robot.motor_front_right.setPower(result[1]);
        robot.motor_rear_left.setPower(result[2]);
        robot.motor_rear_right.setPower(result[3]);


        telemetry.addData("smootingValue", smootingValue);

        if (gamepad2.left_bumper) { // Auf
            robot.servo_grab.setPosition(0.1);
        } else if (gamepad2.right_bumper) { // Zu
            robot.servo_grab.setPosition(0.5);
        }

        robot.motor_lift_left.setPower(-gamepad2.right_stick_y*0.5);
        robot.motor_lift_right.setPower(-gamepad2.right_stick_y*0.5);
        robot.motor_clamp.setPower(gamepad2.left_stick_x);


        telemetry.addData("Smoothing Value: ", smootingValue);
        telemetry.addLine();
        telemetry.addData("Is Blue Back: ", colorTools.isBlue(robot.color_back));
        telemetry.addData("Is Blue Front: ", colorTools.isBlue(robot.color_front));
        telemetry.addLine();
        telemetry.addData("Is Red Back: ", colorTools.isRed(robot.color_back));
        telemetry.addData("Is Red Front: ", colorTools.isRed(robot.color_front));
        telemetry.addLine();
        telemetry.addData("Touch Left: ", robot.touch_left.getState());
        telemetry.addData("Touch Right: ", robot.touch_right.getState());
        telemetry.addLine();
        telemetry.addData("Hue: ", colorTools.showHSV(robot.color_back)); //TO DO: show Hsv values
        telemetry.update();
    }
}