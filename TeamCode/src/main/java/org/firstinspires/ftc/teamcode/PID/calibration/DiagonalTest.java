package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

import java.util.List;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(name = "DiagonalTest", group = "drive")
//@Disabled
public class DiagonalTest extends LinearOpMode {
    public static double DISTANCE = 0; // update later;
    private String TAG = "DiagonalTest";
    SampleMecanumDriveBase _drive = null;

    private void StrafeDiagonalHelper(SampleMecanumDriveBase _drive, Vector2d xy) {
        Trajectory trajectory;
        Pose2d currentPos = _drive.getPoseEstimate();
        TrajectoryBuilder  builder = null;
        if (DriveConstantsPID.USING_STRAFE_DIAGNAL)
            builder = new TrajectoryBuilder(currentPos, DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
        else
            builder = new TrajectoryBuilder(currentPos, DriveConstantsPID.BASE_CONSTRAINTS);

        Pose2d error_pose = _drive.follower.getLastError();

        RobotLogger.dd(TAG, "StrafeDiagonalHelper, xy: %s", xy.toString());
        RobotLogger.dd(TAG, "StrafeDiagonalHelper, currentPos %s, errorPos %s",currentPos.toString(), error_pose.toString());
        double current_x = currentPos.getX();
        double current_y = currentPos.getY();
        double delta_x = xy.getX() - current_x;
        double delta_y = xy.getY() - current_y;

        if (Math.abs(delta_x) > Math.abs(delta_y)) {
            double x_offset = delta_x - delta_y;;
            double squre_offset = delta_y;
            builder.setReversed(false).lineTo(new Vector2d(current_x + x_offset, current_y)).strafeTo(new Vector2d(xy.getX(), xy.getY()));
        }
        else if (Math.abs(delta_x) < Math.abs(delta_y)){
            double y_offset = delta_y - delta_x;
            double squre_offset = delta_x;
            builder.setReversed(false).strafeTo(new Vector2d(current_x, current_y + y_offset)).strafeTo(new Vector2d(xy.getX(), xy.getY()));
        }
        else
        {
            //double y_offset = delta_y - delta_x;
            double squre_offset = delta_x;
            builder.setReversed(false).strafeTo(new Vector2d(xy.getX(), xy.getY()));
        }
        trajectory = builder.build();   //x - 2.812, y + 7.984
        _drive.followTrajectorySync(trajectory);
        RobotLogger.dd(TAG, "StrafeDiagonalHelper, currentPos %s, errorPos %s",currentPos.toString(), error_pose.toString());
    }
    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();  // Transitional PID is used in base class;;
        DISTANCE = DriveConstantsPID.TEST_DISTANCE;

        waitForStart();

        if (isStopRequested()) return;

        if (_drive == null) {
            if (DriveConstantsPID.USING_BULK_READ == false)
                _drive = new SampleMecanumDriveREV(hardwareMap, true);
            else
                _drive = new SampleMecanumDriveREVOptimized(hardwareMap, true);
            _drive.setBrakeonZeroPower(DriveConstantsPID.BRAKE_ON_ZERO);
            _drive.setPoseEstimate(new Pose2d(0, 0, _drive.getExternalHeading()));
        }

        //StrafeDiagonalHelper(_drive, new Vector2d(DriveConstantsPID.TEST_DISTANCE, DriveConstantsPID.TEST_DISTANCE_0));
        Trajectory trajectory = _drive.trajectoryBuilder()
                .strafeTo(new Vector2d(DriveConstantsPID.TEST_DISTANCE, DriveConstantsPID.TEST_DISTANCE_0))
                .build();


        _drive.followTrajectorySync(trajectory);

        Localizer localizer = _drive.getLocalizer();
        if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL && (localizer!=null)) {
            StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer)localizer; // @TODO
            List<Double> odo_positions = t.getWheelPositions();

            RobotLogger.dd(TAG, "odometry positions");
            _drive.print_list_double(odo_positions);
        }

        List<Double> positions = _drive.getWheelPositions();
        RobotLogger.dd(TAG, "wheel positions");
        _drive.print_list_double(positions);
    }
}
