package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.TTArm;
import teamcode.common.TTDriveSystem;
import teamcode.common.TTOpMode;
import teamcode.common.TTRobot;
import teamcode.common.Vector2;

@TeleOp(name = "TT TeleOp")
public class TTTeleOp extends TTOpMode {

    private static final double TURN_SPEED_MODIFIER = 0.6;
    private static final double REDUCED_DRIVE_SPEED = 0.6;

    private TTDriveSystem driveSystem;
    private TTArm arm;

    @Override
    protected void onInitialize() {
    }

    @Override
    protected void onStart() {
        TTRobot robot = getRobot();
        //driveSystem = robot.getDriveSystem();
        arm = robot.getArm();

        while (opModeIsActive()) {
            update();
        }
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

    private void armUpdate(){
        if(gamepad2.y){
            arm.armLift();
        }else if(gamepad2.a){
            arm.armLower();
        }
        if(gamepad2.x && arm.getClawPos() == 1){
            arm.rotateClaw(0);
        }else if (gamepad2.x && arm.getClawPos() == 0){
            arm.rotateClaw(1);
        }
        if (gamepad2.b && arm.getWristPos() < 0.7) {
            arm.rotateWrist(0);
        }else if (gamepad2.b && arm.getWristPos() < 0.1){
            arm.rotateWrist(0.7);
        }
    }

}
