package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.structurebuilder.StructureConstructor;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.prefab.TwoByTwoByFive;
import org.firstinspires.ftc.teamcode.hardware.Elevator;
import org.firstinspires.ftc.teamcode.hardware.FoundationGrabber;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;

@TeleOp (name = "Competition Teleop")
public class CompetitionTeleop extends LinearOpMode {

    private SampleMecanumDriveREVOptimized drive;
    private Elevator elevator;
    private Intake intake;
    private FoundationGrabber foundationGrabber;

    private StructureConstructor structureConstructor;

    @Override
    public void runOpMode() {

        drive = SampleMecanumDriveREVOptimized.getInstance(hardwareMap);
        elevator = Elevator.getInstance(hardwareMap);
        intake = Intake.getInstance(hardwareMap);
        foundationGrabber = FoundationGrabber.getInstance(hardwareMap);

        structureConstructor = new StructureConstructor(new TwoByTwoByFive().toStructure());

        waitForStart();
        intake.release();
        while (!isStopRequested()) {

            //Foundation Grabber
            if(gamepad1.a){
                foundationGrabber.setCurrentPosition(FoundationGrabber.Positions.DOWN_LEFT);
            } else{
                foundationGrabber.setCurrentPosition(FoundationGrabber.Positions.UP_LEFT);
            }

            //Intake Control
            if(gamepad2.a){
                intake.setGrabbing();
            }
            if(gamepad2.b){
                intake.open();
            }

            //Elevator Control
            if(gamepad2.left_stick_y !=0){
                elevator.setMotorPowers(gamepad2.left_stick_y);
                elevator.setDriverControlled();
            } else {
                elevator.stop();
            }

            elevator.update();

            if((gamepad1.left_stick_y < 0.05 && gamepad1.left_stick_y > -0.05) || (gamepad1.left_stick_x < 0.05 && gamepad1.left_stick_x > -0.05) || (gamepad1.right_stick_x < 0.05 && gamepad1.right_stick_x > -0.05)) {
                //Drive Control
                drive.setDrivePower(new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        (-gamepad1.right_stick_x)/1.5)
                );
            } else {
                drive.setDrivePower(new Pose2d(0,0,0));
            }

            updateTelemetry();

        }
    }

        public void updateTelemetry(){
            Pose2d driveTrainLocation = drive.getPoseEstimate();
            telemetry.addData("Drivetrain X: ", driveTrainLocation.getX());
            telemetry.addData("Drivetrain Y: ", driveTrainLocation.getY());
            telemetry.addData("Drivetrain Heading: ", Math.toDegrees(driveTrainLocation.getHeading()));

            telemetry.addData("Elevator Height: ", elevator.getRelativeHeight());

            telemetry.update();
        }
}
