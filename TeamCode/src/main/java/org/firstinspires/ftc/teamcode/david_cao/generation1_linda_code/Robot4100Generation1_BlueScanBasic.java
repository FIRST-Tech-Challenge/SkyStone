package org.firstinspires.ftc.teamcode.david_cao.generation1_linda_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot3DPositionIndicator;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotCamera;
import org.darbots.darbotsftclib.season_specific.skystone.navigation.SkyStoneNavigation;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

@Autonomous(group = "4100", name="4100Gen1Auto-BlueScanBasic")
public class Robot4100Generation1_BlueScanBasic extends DarbotsBasicOpMode<Robot4100Generation1_LindaCore> {
    private Robot4100Generation1_LindaCore m_RobotCore;
    private SkyStoneNavigation m_Navigation;
    private int ScanResult = 0;

    @Override
    public Robot4100Generation1_LindaCore getRobotCore() {
        return m_RobotCore;
    }

    @Override
    public void hardwareInitialize() {
        this.m_RobotCore = new Robot4100Generation1_LindaCore(this.hardwareMap);
        RobotCamera Camera = new RobotOnPhoneCamera(this,Robot4100Generation1_Settings.AUTONOMOUS_TENSORFLOW_PREVIEW, RobotOnPhoneCamera.PhoneCameraDirection.Back, Robot4100Common.VUFORIA_LICENSE);
        Robot3DPositionIndicator CameraPosition = new Robot3DPositionIndicator(Robot4100Generation1_Settings.AUTONOMOUS_CAMERAPOSONPHONE);
        this.m_Navigation = new SkyStoneNavigation(CameraPosition,Camera);
        this.m_Navigation.setActivated(true);
    }

    @Override
    public void hardwareDestroy() {
        this.m_Navigation.setActivated(false);
    }

    @Override
    public void RunThisOpMode() {
        this.getRobotCore().getChassis().setGyroGuidedDriveEnabled(true);
        this.getRobotCore().getChassis().setGyroGuidedDrivePublicStartingAngleEnabled(true);
        this.getRobotCore().getChassis().updateGyroGuidedPublicStartingAngle();

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                40,
                0.6
        ));
        if(!waitForDrive()){
            return;
        }

        double firstScanExtraDistance = 0;
        int[] stepResult = {1,2,3};
        for(int i=0; i<3; i++){
            boolean foundStone = false;
            if(this.m_Navigation.getDarbotsRobotAxisStonePosition() != null || i == 2){
                foundStone = true;
            }
            if(foundStone){
                this.ScanResult = stepResult[i];
                break;
            }
            firstScanExtraDistance += Robot4100Generation1_Settings.AUTONOMOUS_LENGTH_FOR_EACH_STONE;
            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                    -Robot4100Generation1_Settings.AUTONOMOUS_LENGTH_FOR_EACH_STONE,
                    0.25
            ));
            if(!waitForDrive()){
                return;
            }
        }
        double firstScanZOffset = 0;
        Robot3DPositionIndicator firstScanStonePosition = this.m_Navigation.getDarbotsRobotAxisStonePosition();
        if(firstScanStonePosition != null){
            firstScanZOffset = firstScanStonePosition.getZ();
        }
        firstScanExtraDistance -= firstScanZOffset;
        firstScanExtraDistance -= Robot4100Generation1_Settings.AUTONOMOUS_DISTANCE_BETWEEN_PHONE_AND_STONEGRABBER;
        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                firstScanZOffset + Robot4100Generation1_Settings.AUTONOMOUS_DISTANCE_BETWEEN_PHONE_AND_STONEGRABBER,
                0.25
        ));
        if(!waitForDrive()){
            return;
        }
        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                75-40+10,
                0.2
        ));
        if(!waitForDrive()){
            return;
        }

        this.getRobotCore().setAutonomousDragStoneServoToDrag(true);
        sleep(400);

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                -50,
                0.5
        ));
        if(!waitForDrive()){
            return;
        }

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                130 + firstScanExtraDistance,
                1.0
        ));
        if(!waitForDrive()){
            return;
        }

        this.getRobotCore().setAutonomousDragStoneServoToDrag(false);
        sleep(400);

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                -35,
                0.5
        ));
        if(!waitForDrive()){
            return;
        }
    }
}
