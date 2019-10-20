package org.firstinspires.ftc.teamcode.david_cao.testcases;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot3DPositionIndicator;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotCamera;
import org.darbots.darbotsftclib.season_specific.skystone.navigation.SkyStoneNavigation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

public class ConceptDarbotsStoneDetection extends DarbotsBasicOpMode {
    private SkyStoneNavigation m_Nav;

    @Override
    public RobotCore getRobotCore() {
        return null;
    }

    @Override
    public void hardwareInitialize() {
        RobotCamera mCamera = new RobotOnPhoneCamera(this,true, RobotOnPhoneCamera.PhoneCameraDirection.Back, Robot4100Common.VUFORIA_LICENSE);
        Robot3DPositionIndicator CameraPosition = new Robot3DPositionIndicator(
                0,
                0,
                0,
                -90,
                90,
                0
        );
        m_Nav = new SkyStoneNavigation(CameraPosition,mCamera);
        m_Nav.setActivated(true);
    }

    @Override
    public void hardwareDestroy() {
        m_Nav.setActivated(false);
    }

    @Override
    public void RunThisOpMode() {
        waitForStart();
        if(this.opModeIsActive()){
            while(opModeIsActive()){
                Robot3DPositionIndicator StonePosition = m_Nav.getDarbotsRobotAxisStonePosition();
                if(StonePosition != null){
                    telemetry.addLine("stonePosition")
                            .addData("Status","Visible")
                            .addData("X",StonePosition.getX())
                            .addData("Z",StonePosition.getZ())
                            .addData("Y",StonePosition.getY())
                            .addData("XRot",StonePosition.getRotationX())
                            .addData("ZRot",StonePosition.getRotationZ())
                            .addData("YRot",StonePosition.getRotationY());
                }else{
                    telemetry.addLine("stonePosition")
                            .addData("Status","Invisible");
                }
                telemetry.update();
            }
        }
    }
}
