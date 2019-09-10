package org.firstinspires.ftc.teamcode.RoverRuckus.RR2.ethan_code;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto.TensorFlowMineralDetection;
import org.firstinspires.ftc.teamcode.RoverRuckus.RR2.RR2;
@Deprecated
@Disabled
@Autonomous
public class drive extends AutoBase {
    public DcMotor fLeft;
    public DcMotor fRight;
    public DcMotor bLeft;
    public DcMotor bRight;

    protected BNO055IMU imu;

    private Orientation angles;

    @Override
    public void runOpMode() {
        // Initialize motors and robot
        initOpMode();

        waitForStart();

        // Move the robot 12 inches, slowing down linearly
        move(4000);
    }

    protected void initOpMode() {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotor.Direction.FORWARD);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.REVERSE);


        // Init imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
    }

    protected void move(int distance) {
        int halfway = distance/2;
        double power;
        int x = 0;

        // Initiate drive motors
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the target position of the drive motors. Divide by 0.028 to convert inches to encoder ticks
        fLeft.setTargetPosition(distance);
        fRight.setTargetPosition(distance);
        bLeft.setTargetPosition(distance);
        bRight.setTargetPosition(distance);

        // While the motors have not reached the target position, set each individual motor's power proportional to how much further it has to go.
        while (fLeft.isBusy() || fRight.isBusy() || bLeft.isBusy() || bRight.isBusy()) {
            x++;

            telemetry.addData("x",x);
            telemetry.addData("halfway",halfway);

            if (fLeft.getCurrentPosition() <= halfway) {
                power = (double) fLeft.getCurrentPosition() / halfway;
                telemetry.addData("power",power);
                if (power < .05) {
                    power = .05;
                }
            } else {
                power = (double) (distance - fLeft.getCurrentPosition()) / distance;
            }

            telemetry.addData("fLeft",fLeft.getCurrentPosition());
            telemetry.addData("fRight",fRight.getCurrentPosition());
            telemetry.addData("bLeft",fLeft.getCurrentPosition());
            telemetry.addData("bRight",bRight.getCurrentPosition());

            telemetry.addData("fLeft Power", power);

            telemetry.addData("fLeft Target", fLeft.getTargetPosition());

            telemetry.update();

            fLeft.setPower(power);
            fRight.setPower(power);
            bLeft.setPower(power);
            bRight.setPower(power);
        }

        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

    protected void turn() {

    }
}
