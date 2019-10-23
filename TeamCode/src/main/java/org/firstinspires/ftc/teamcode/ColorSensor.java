package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

    @Autonomous(name = "ColorSensor (Blocks to Java)", group = "")
    public class ColorSensor extends LinearOpMode {

        private DcMotor motorDriveLeft;
        private DcMotor motorDriveRight;
        private ColorSensor sensorOTJ;


        @Override
        public void runOpMode() {
            int CurrentColor;

            motorDriveLeft = hardwareMap.dcMotor.get("motorDriveLeft");
            motorDriveRight = hardwareMap.dcMotor.get("motorDriveRight");
            sensorOTJ = (ColorSensor) hardwareMap.colorSensor.get("sensorOTJ");

            // Put initialization blocks here.
            motorDriveLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            sleep(500);
            waitForStart();
            motorDriveLeft.setPower(.714285);
            motorDriveRight.setPower(1);
            while (opModeIsActive()) {
                CurrentColor = Color.rgb(Color.red(), Color.green(), Color.blue());
                if (JavaUtil.colorToSaturation(CurrentColor) >= 0.3 && JavaUtil.colorToHue(CurrentColor) > 200 && JavaUtil.colorToHue(CurrentColor) < 285) {
                    motorDriveLeft.setPower(0);
                    motorDriveRight.setPower(0);
                }
                telemetry.update();
            }
        }
    }


