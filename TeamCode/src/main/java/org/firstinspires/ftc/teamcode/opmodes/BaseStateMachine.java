package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;

import java.util.EnumMap;


public abstract class BaseStateMachine extends BaseOpMode {


        private VuforiaTrackable skystone;
        private static final float mmPerInch = 25.4f;


        protected Vuforia.CameraChoice currentCamera;

        protected Vuforia vuforia;

        protected VuforiaTrackable stoneTarget;

        protected VuforiaTrackable wallTarget;

        protected VuforiaTrackables targetsSkyStone;

        protected DriveSystem driveSystem;

        @Override
        public void init() {
            super.init();
            driveSystem = super.driveSystem;
        }

}