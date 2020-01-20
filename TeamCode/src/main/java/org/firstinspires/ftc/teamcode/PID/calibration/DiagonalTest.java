package org.firstinspires.ftc.teamcode.PID.calibration;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.All.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Experimental.Units.Vector;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;

@Config
@Autonomous(name = "DiagonalTest", group = "drive")
public class DiagonalTest extends LinearOpMode{

    public static double xOffset = 0;
    public static double yOffset = 0;

    public static Vector2d desiredPoint = new Vector2d(xOffset, yOffset);
    public static Vector2d originalPoint = new Vector2d(0,0);
    private String TAG = "DiagonalTest";
    @Override
    public void runOpMode(){
        DriveConstantsPID.updateConstantsFromProperties();
        SampleMecanumDriveBase drive = null;
        if (DriveConstantsPID.USING_BULK_READ == false)
            drive = new SampleMecanumDriveREV(hardwareMap, false); //base constants not strafe
        else
            drive = new SampleMecanumDriveREVOptimized(hardwareMap, false); // base constant not strafe
        drive.setBrakeonZeroPower(DriveConstantsPID.BRAKE_ON_ZERO);
        HardwareMap hwMap = new HardwareMap(hardwareMap);
        RobotLog.dd(TAG, "trajectoryBuilder diagonal, X: "+Double.toString(xOffset)+" Y: "+Double.toString(yOffset));
        Trajectory trajectory = drive.trajectoryBuilder().strafeTo(desiredPoint).strafeTo(originalPoint).build();

        waitForStart();
        for(int i=0; i<4; i++){
            if(isStopRequested()) return;
            drive.followTrajectorySync(trajectory);
            Localizer localizer = drive.getLocalizer();

            if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL && (localizer!=null)) {
                StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer)localizer; // @TODO
                List<Double> odo_positions = t.getWheelPositions();

                RobotLog.dd(TAG, "odometry positions");
                drive.print_list_double(odo_positions);
            }

            List<Double> positions = drive.getWheelPositions();
            RobotLog.dd(TAG, "wheel positions");
            drive.print_list_double(positions);
        }

    }

}
