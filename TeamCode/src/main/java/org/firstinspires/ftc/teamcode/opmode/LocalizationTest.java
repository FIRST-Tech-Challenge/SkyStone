package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.StructureConstructor;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.prefab.TwoByTwoByFive;
import org.firstinspires.ftc.teamcode.hardware.Elevator;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.FoundationGrabber;
import org.firstinspires.ftc.teamcode.util.Button;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
@Disabled
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        FoundationGrabber foundationGrabber = new FoundationGrabber(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        Elevator elevator = Elevator.getInstance(hardwareMap);

        StructureConstructor structureConstructor = new StructureConstructor(TwoByTwoByFive.toStructure());

        DcMotor leftEncoder = hardwareMap.dcMotor.get("leftFront");
        DcMotor frontEncoder = hardwareMap.dcMotor.get("leftRear");

        //Control Mapping
        Button elevatorUpButton = new Button(gamepad2,Button.ListenButton.A);
        Button elevatorDownButton = new Button(gamepad2,Button.ListenButton.B);

        waitForStart();
        while (!isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            drive.setDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x)
            );

            drive.update();
            if(gamepad1.a){
                foundationGrabber.setCurrentPosition(FoundationGrabber.Positions.DOWN_LEFT);
            } else{
                foundationGrabber.setCurrentPosition(FoundationGrabber.Positions.UP_LEFT);
            }

            elevatorUpButton.update();
            elevatorDownButton.update();


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("Motor Ticks (Left Encoder):" , leftEncoder.getCurrentPosition());
            telemetry.addData("Motor Ticks (Front Encoder): ", frontEncoder.getCurrentPosition());
            telemetry.update();

            fieldOverlay.setStroke("#3F51B5");
            fieldOverlay.fillCircle(poseEstimate.getX(), poseEstimate.getY(), 3);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
