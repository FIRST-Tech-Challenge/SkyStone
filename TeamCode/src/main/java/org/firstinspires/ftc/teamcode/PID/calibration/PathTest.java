package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.FieldPosition;
import org.firstinspires.ftc.teamcode.Autonomous.Path;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.TeleOp.TeleopConstants;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class PathTest extends LinearOpMode {
    private Trajectory trajectory;
    private BaseTrajectoryBuilder builder, strafe_builder;
    private Pose2d current_pose;
    private String TAG = "PathTest";
    private SampleMecanumDriveBase _drive = null;
    private HardwareMap hwMap;
    private Path path;
    private FieldPosition fieldPosition = null;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();
        // do we need this??? all this are HW settings to enable ARM actions;
        //hwMap.liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //hwMap.liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hwMap.liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hwMap.liftOne.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        int[] skystonePositions = new int[2];
        skystonePositions[0] = (int) DriveConstantsPID.TEST_SKY_STONE_POSITION;

        if (DriveConstantsPID.USING_BULK_READ == false)
            _drive = new SampleMecanumDriveREV(hardwareMap, false);
        else
            _drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);

        RobotLogger.dd(TAG, "unit test for path (RED QUARY), ARM actions?" + Integer.toString(DriveConstantsPID.ENABLE_ARM_ACTIONS?1:0));
        Pose2d startingPos = new Pose2d(new Vector2d(-34.752, -63.936), Math.toRadians(0));
        hwMap = new HardwareMap(hardwareMap);
        fieldPosition = FieldPosition.RED_QUARY;

        waitForStart();

        if (isStopRequested()) return;

        path = new Path(hwMap, this, _drive, startingPos, hardwareMap, null);
        path.RedQuary(skystonePositions);
        RobotLogger.dd(TAG, "----------done --------------------- unit test for path (RED QUARY)");
    }
}
