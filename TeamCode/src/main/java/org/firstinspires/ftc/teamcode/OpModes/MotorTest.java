package org.firstinspires.ftc.teamcode.OpModes;

/*
@TeleOp(name = "Motor Test", group = "Teleop")
public class MotorTest extends LinearOpMode {
    long timeTo = System.currentTimeMillis();

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Chassis chassis = new Chassis(hardwareMap);
        //chassis.reverseMotors(new DcMotor[]{chassis.frontRight, chassis.backRight});
        robot.setChassis(chassis);
        DcMotor motor = hardwareMap.dcMotor.get("test");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("hit", robot.chassis.backRight.getCurrentPosition());
            if (gamepad1.a) {
                chassis.runRotations(.5, .25);
            }
            if (gamepad1.b) {
                robot.arm.run(3);
            }
            if (gamepad1.x) {
                robot.intake.setMainToHighPosition();
            }
            if (gamepad1.y) {
                robot.arm.setArm(4);
                chassis.runRotations(.5, -.25);
            }
            //Other unit test code if you want
            telemetry.update();
            }
            }
 */