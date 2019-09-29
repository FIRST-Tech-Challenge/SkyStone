package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * It's teleop... yeah
 */
@TeleOp(name="Teleop", group="Teleop")
public class Teleop extends OpMode {

    private Hardware robot;
    private MecanumDrive driveTrain;


    /**
     * Instantiates objects
     */
    public Teleop() {
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);
    }

    /**
     * Method run on init to initialize hardware
     */
    public void init() {
        // Hardware map setup
        robot.init(hardwareMap);

        // Gyro setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters);
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Odometry setup
        robot.resetEncoders();

    }

    /**
     * Loop during init phase
     * Tells you if the gyro is calibrated
     */
    @Override
    public void init_loop() {
        telemetry.addData("imu calabration", robot.imu.isGyroCalibrated());
    }

    /**
     * The loop played during the game
     */
    @Override
    public void loop() {
        robot.bulkData = robot.expansionHub.getBulkInputData();
        robot.updatePosition();
        telemetry.addData("xPos", robot.x);
        telemetry.addData("yPos", robot.y);
        telemetry.addData("theta", robot.theta);
        telemetry.addData("theta degrees", Math.toDegrees(robot.theta));
        telemetry.addLine("==========");
        telemetry.addData("total left traveled(cm)", robot.rightOdomTraveled);
        telemetry.addData("total right traveled(cm)", robot.leftOdomTraveled);
        telemetry.addData("total center traveled(cm)", robot.centerOdomTraveled);
        telemetry.addLine("==========");
        telemetry.addData("leftEncoder value", robot.leftEncoderPos);
        telemetry.addData("rightEncoder value", robot.rightEncoderPos);
        telemetry.addData("centerEncoder value", robot.centerEncoderPos);
        telemetry.update();

        // Reset robot position = X
        if(gamepad1.x){
            robot.resetPosition();
            robot.resetEncoders();
        }

        if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0) {
            driveTrain.brakeMotors();
        } else {
            driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
    }
}
