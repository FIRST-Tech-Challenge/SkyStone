package DemoCodes.impl;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import DemoCodes.common.DemoArm;
import DemoCodes.common.DemoDrive;
import DemoCodes.common.TTOpMode;

@TeleOp(name = "Demo TeleOp")
public class DemoTeleOp extends TTOpMode {

    private static final double TURN_SPEED_MODIFIER = 0.3;
    private static final double REDUCED_DRIVE_SPEED = 0.4;
    private DemoDrive driveSystem;
    private DemoArm arm;

    @Override
    protected void onInitialize() {
        driveSystem = new DemoDrive(hardwareMap);
        arm = new DemoArm(hardwareMap);
    }

    @Override
    protected void onStart() {
        while (opModeIsActive()) {
            driveUpdate();
            armUpdate();
        }
    }

    protected void onStop() {
    }

    private void driveUpdate() {
        double vertical = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        driveSystem.continuous(vertical, turn);
    }

    private void armUpdate(){
        while(gamepad1.dpad_up){
            arm.raise();
        }
        while(gamepad1.dpad_down){
            arm.lower();
        }
        arm.stop();
    }

}