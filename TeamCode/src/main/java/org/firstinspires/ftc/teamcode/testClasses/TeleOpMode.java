package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Controller;

import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.subsystems.RobotMap.ChassisMotor;

@TeleOp(name = "TeleOpMode", group = "Teleop")
public class TeleOpMode extends LinearOpMode {
    public void runOpMode() {
        Chassis chassis;
        chassis = new Chassis(hardwareMap, new HashMap<ChassisMotor, String>() {{
            put(ChassisMotor.FRONT_LEFT, "front_left_drive");
            put(ChassisMotor.FRONT_RIGHT, "front_right_drive");
            put(ChassisMotor.BACK_LEFT, "back_left_drive");
            put(ChassisMotor.BACK_RIGHT, "back_right_drive");
        }});
        DcMotor hook = hardwareMap.dcMotor.get("hook");
        //DcMotor leftIntake = hardwareMap.dcMotor.get("left_intake");
        //DcMotor rightIntake = hardwareMap.dcMotor.get("right_intake");
        telemetry.addData("Init", "v:1.0");
        waitForStart();

        while (opModeIsActive()) {
            Controller controller = new Controller(gamepad1);

            //
            //Hook test
            if (gamepad1.a == true) {
                hook.setPower(1);
            }
            else if (gamepad1.b == true) {
                hook.setPower(-1);
            }
            else {
                hook.setPower(0);
            }
            /*
            //Intake test
            if (gamepad1.x == true) {
                leftIntake.setPower(1);
                rightIntake.setPower(-1);
            }
            else if (gamepad1.y == true) {
                leftIntake.setPower(-1);
                rightIntake.setPower(1);
            }
            else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }
            */
            //Chassis test
            final double stickRadius = Math.hypot(controller.getLeftStickX(), controller.getLeftStickY());//Flip Y stick
            final double targetAngle = Math.atan2(controller.getLeftStickY(), controller.getLeftStickX()) - Math.PI / 4;
            final double turnPower = controller.getRightStickX();
            final double frontLeftPower = stickRadius * Math.cos(targetAngle) + turnPower;
            final double frontRightPower = stickRadius * Math.sin(targetAngle) - turnPower;
            final double backLeftPower = stickRadius * Math.sin(targetAngle) + turnPower;
            final double backRightPower = stickRadius * Math.cos(targetAngle) - turnPower;
            HashMap<ChassisMotor, Double> chassisPowers = new HashMap<>();
            chassisPowers.put(ChassisMotor.FRONT_LEFT, frontLeftPower);
            chassisPowers.put(ChassisMotor.FRONT_RIGHT, frontRightPower);
            chassisPowers.put(ChassisMotor.BACK_LEFT, backLeftPower);
            chassisPowers.put(ChassisMotor.BACK_RIGHT, backRightPower);
            chassis.setMotors(chassisPowers);

            //Telemetry
            telemetry.update();
        }
    }
}  
