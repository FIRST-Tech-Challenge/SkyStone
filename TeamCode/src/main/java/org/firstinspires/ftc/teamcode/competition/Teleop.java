package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * It's teleop... yeah
 */
@TeleOp(name="Teleop", group="Teleop")
public class Teleop extends OpMode {

    private Hardware robot;
    private MecanumDrive driveTrain;

    private boolean brake;
    private boolean resettingPos = false;

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
        robot.updatePosition();
        telemetry.addData("xPos", robot.x);
        telemetry.addData("yPos", robot.y);
        telemetry.addData("theta", robot.theta);
        telemetry.update();

        if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0) {
            driveTrain.brakeMotors();
            brake = true;
        } else {
            if(brake) {
                robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }

        if(gamepad1.x && !resettingPos){
            robot.resetPosition();
            resettingPos = true;
        } else if (!gamepad1.x){
            resettingPos = false;
        }

        /*
          TODO:
            Autosave position data every run-through of the main loop
            Make a button which will load position data (for robot restarts mid match)
         */
    }

    /**
     * Logs recording data from the robot to an external file
     * - X, Y, Theta
     * - Motor/servo values
     */
    private void logRecording(){

    }
}
