package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.TimerTask;

import teamcode.common.League1TTArm;
import teamcode.common.TTArm;
import teamcode.common.TTDriveSystem;
import teamcode.common.TTOpMode;
import teamcode.common.TTRobot;
import teamcode.common.Vector2;

@TeleOp(name = "TT TeleOp")
public class TTTeleOp extends TTOpMode {

    private static final double TURN_SPEED_MODIFIER = 0.3;
    private static final double REDUCED_DRIVE_SPEED = 0.4;
    private static final double CLAW_COOLDOWN_SECONDS = 0.5;

    private TTDriveSystem driveSystem;
    private League1TTArm arm;

    private boolean canUseClaw;

    @Override
    protected void onInitialize() {
        driveSystem = new TTDriveSystem(hardwareMap);
        arm = new League1TTArm(hardwareMap);
        canUseClaw = true;
    }

    @Override
    protected void onStart() {

        TTRobot robot = getRobot();
        //driveSystem = robot.getDriveSystem();
        new ArmInputListener().start();
        while (opModeIsActive()) {
            //        driveUpdate();
        }
    }

    @Override
    protected void onStop() {

    }

    private void update() {
        //driveUpdate();
        armUpdate();
    }

//    private void driveUpdate() {
//        double vertical = gamepad1.right_stick_y;
//        double horizontal = gamepad1.right_stick_x;
//        double turn = gamepad1.left_stick_x * TURN_SPEED_MODIFIER;
//        Vector2 velocity = new Vector2(vertical, horizontal);
//        if (!gamepad1.right_bumper) {
//            velocity = velocity.multiply(REDUCED_DRIVE_SPEED);
//        }
//        driveSystem.continuous(velocity, turn);
//    }

    private void armUpdate() {
        if (gamepad2.y) {
            telemetry.addData("status", "reached armLift");
            telemetry.update();
            arm.liftContinuous(1);
            telemetry.addData("status", "cleared armLift");
            telemetry.update();
        } else if (gamepad2.a) {
            arm.lower(1);
        }
    }



        private void armUpdate() {
            if (gamepad1.y) {
                arm.raise(0.5);
            } else if (gamepad1.a) {
                arm.lower(0.5);
            } else if (gamepad1.b) {
                arm.liftTimed(0.75, 0.5);
            }
            if (gamepad1.dpad_up) {
                arm.liftContinuous(0.5);
            }
            if (gamepad1.dpad_down) {
                arm.liftContinuous(-0.5);
            } else {
                arm.liftContinuous(0.0);
            }
            if (gamepad1.x && canUseClaw) {
                if (arm.clawIsOpen()) {
                    arm.closeClaw();
                } else {
                    arm.openClaw();
                }
                driveSystem.continuous(velocity, turn);
            }

            private class ArmInputListener extends Thread {

                @Override
                public void run() {
                    while (opModeIsActive()) {
                        armUpdate();
                    }
                }

                private void armUpdate() {
                    if (gamepad1.y) {
                        arm.raise(0.5);
                    } else if (gamepad1.a) {
                        arm.lower(0.5);
                    } else if (gamepad1.b) {
                        arm.liftTimed(1.0, 0.5);
                    }
                    if (gamepad1.dpad_up) {
                        arm.liftContinuous(0.5);
                    }
                    if (gamepad1.dpad_down) {
                        arm.liftContinuous(-0.5);
                    } else {
                        arm.liftContinuous(0.0);
                    }
                    if (gamepad1.x && canUseClaw) {
                        if (arm.clawIsOpen()) {
                            arm.closeClaw();
                        } else {
                            arm.openClaw();
                        }
                        clawCooldown();
                    }
                }

                private void clawCooldown() {
                    canUseClaw = false;
                    TimerTask enableClaw = new TimerTask() {
                        @Override
                        public void run() {
                            canUseClaw = true;
                        }
                    };
                    getTimer().schedule(enableClaw, (long) (CLAW_COOLDOWN_SECONDS * 1000));
                }

            }

        }
