package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Crane;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

@TeleOp(name = "TeleOp", group = "Concept")



public class BasicTeleOp extends TeleOpControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        boolean yToggle = false;

        setup(runtime, Crane.setupType.drive, Crane.setupType.claw, Crane.setupType.foundation);

        while (opModeIsActive()){
            standardGamepadData();


            if (gamepad2.y) {
                rob.rightServo.setPosition(rob.rightServo.getPosition() + 0.01);
                rob.leftServo.setPosition(rob.leftServo.getPosition() + 0.01);
            } else if (gamepad2.a) {
                rob.rightServo.setPosition(rob.rightServo.getPosition() - 0.01);
                rob.leftServo.setPosition(rob.leftServo.getPosition() - 0.01);
            }

            if(gamepad2.b){
                rob.rotationservo.setPosition(rob.rotationservo.getPosition() - 0.0025);
            }else if (gamepad2.x) {
                rob.rotationservo.setPosition(rob.rotationservo.getPosition() + 0.0025);
            }

            if(gamepad1.dpad_right){
                rob.foundationServo.setPosition(0.1);
            }else if(gamepad1.dpad_left){
                rob.foundationServo.setPosition(0.5);
            }

            if (gamepad1.y){
                yToggle = !yToggle;
            }

            telemetry.addData("power", fb);
            telemetry.update();

            if (!yToggle) {
                if (g(0)) {
                    rob.driveTrainMovement(fb, Crane.movements.forward);
                } else if (g(2)) {
                    rob.driveTrainMovement(fb, Crane.movements.backward);
                } else if (g(3)) {
                    rob.driveTrainMovement(rl, Crane.movements.right);
                } else if (g(1)) {
                    rob.driveTrainMovement(rl, Crane.movements.left);
                }
                else if (g(4)) {
                    rob.driveTrainMovement(diagonalSpeed, Crane.movements.tr);
                }else if (g(5)) {
                    rob.driveTrainMovement(diagonalSpeed, Crane.movements.tl);
                }else if (g(6)) {
                    rob.driveTrainMovement(diagonalSpeed, Crane.movements.bl);
                }else if (g(7)) {
                    rob.driveTrainMovement(diagonalSpeed, Crane.movements.br);
                }
                else if (g(8)) {
                    rob.driveTrainMovement(0.8, Crane.movements.ccw);
                } else if (g(9)) {
                    rob.driveTrainMovement(0.8, Crane.movements.cw);
                } else {
                    rob.stopDrivetrain();
                }
            }
            else {
                if (g(0)) {
                    rob.driveTrainMovement(0.2, Crane.movements.forward);
                } else if (g(2)) {
                    rob.driveTrainMovement(0.2, Crane.movements.backward);
                } else if (g(3)) {
                    rob.driveTrainMovement(0.2, Crane.movements.right);
                } else if (g(1)) {
                    rob.driveTrainMovement(0.2, Crane.movements.left);
                }
                else if (g(4)) {
                    rob.driveTrainMovement(0.2, Crane.movements.tr);
                }else if (g(5)) {
                    rob.driveTrainMovement(0.2, Crane.movements.tl);
                }else if (g(6)) {
                    rob.driveTrainMovement(0.2, Crane.movements.bl);
                }else if (g(7)) {
                    rob.driveTrainMovement(0.2, Crane.movements.br);
                }
                else if (g(8)) {
                    rob.driveTrainMovement(0.2, Crane.movements.ccw);
                } else if (g(9)) {
                    rob.driveTrainMovement(0.2, Crane.movements.cw);
                } else {
                    rob.stopDrivetrain();
                }
            }

            if(gamepad2.dpad_left){
                if(!rob.flimit.getState()) {
                    rob.rack.setPower(0.2);
                }
            }else if(gamepad2.dpad_right) {
                if (!rob.blimit.getState()){
                    rob.rack.setPower(-0.2);
                }
            }else{
                rob.rack.setPower(0);
            }

            if(gamepad2.dpad_up){
                rob.rightLinear.setPower(0.6);
                rob.leftLinear.setPower(0.6);
            }else if(gamepad2.dpad_down){
                rob.rightLinear.setPower(-0.6);
                rob.leftLinear.setPower(-0.6);
            }else {
                rob.rightLinear.setPower(0);
                rob.leftLinear.setPower(0);
            }

        }
    }
}
