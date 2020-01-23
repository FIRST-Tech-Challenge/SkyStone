
package org.firstinspires.ftc.teamcode.PID.calibration;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;


import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "StraightTestV0", group = "drive")
public class StraightTestV0 extends LinearOpMode {
    public static double DISTANCE = DriveConstantsPID.TEST_DISTANCE;
    private String TAG = "StraightTest";
    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();
        DISTANCE = DriveConstantsPID.TEST_DISTANCE;
        SampleMecanumDriveBase drive = null;
        if (DriveConstantsPID.USING_BULK_READ == false)
            drive = new SampleMecanumDriveREV(hardwareMap, false);
        else
            drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);
        drive.setBrakeonZeroPower(DriveConstantsPID.BRAKE_ON_ZERO);
        RobotLogger.dd(TAG, "trajectoryBuilder forward, DISTANCE: "+Double.toString(DISTANCE));
        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
        Localizer localizer = drive.getLocalizer();
        if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL && (localizer!=null)) {
            StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer)localizer; // @TODO
            List<Double> odo_positions = t.getWheelPositions();

            RobotLogger.dd(TAG, "odometry positions");
            drive.print_list_double(odo_positions);
        }

        List<Double> positions = drive.getWheelPositions();
        RobotLogger.dd(TAG, "wheel positions");
        drive.print_list_double(positions);

    }
}
