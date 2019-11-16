package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name = "TurnTest", group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareMap map = new HardwareMap(hardwareMap);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        //HardwareMap.track.resetEncoders();
        //HardwareMap.track.encoders(true);

        waitForStart();

        if (isStopRequested()) return;

        drive.turnSync(Math.toRadians(ANGLE));

        while(opModeIsActive()){
            /*telemetry.addData("LeftForward", map.leftForward.getVoltage());
            telemetry.addData("RightForward", map.rightForward.getVoltage());
            telemetry.addData("Sideways", map.sideways.getVoltage());

            telemetry.addData("LeftForward", HardwareMap.track.getEncoderTicks().get(0));
            telemetry.addData("RightForward", HardwareMap.track.getEncoderTicks().get(1));
            telemetry.addData("Sideways", HardwareMap.track.getEncoderTicks().get(2));
            telemetry.update();*/
        }
    }
}
