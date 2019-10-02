package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.components.Vuforia;


@TeleOp(name="Vuforia Test OpMode", group="Autonomous")
public class VuforiaTest extends LinearOpMode {
    private static final float mmPerInch = 25.4f;

    @Override
    public void runOpMode() {
        Vuforia vuforia = new Vuforia(this.hardwareMap, Vuforia.CameraChoice.PHONE_FRONT);
        waitForStart();
        VectorF translation;

        while (opModeIsActive()) {
            if (vuforia.isAnyTargetVisible()) {
                translation = vuforia.getRobotPosition();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                telemetry.update();
            }
        }
    }
}
