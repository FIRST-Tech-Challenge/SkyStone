package org.firstinspires.ftc.teamcode.david_cao.generation1_linda_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.GyroGuidedTurn;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotCamera;
import org.darbots.darbotsftclib.season_specific.skystone.tfod_detection.SkyStoneStoneDifferentiation;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.util.ArrayList;

@Autonomous(name = "4100Gen1Auto-RedScanDouble",group="4100")
public class Robot4100Generation1_RedScanDouble extends DarbotsBasicOpMode {
    private SkyStoneStoneDifferentiation m_SkyStoneDetection;
    private Robot4100Generation1_LindaCore m_RobotCore;
    private float m_OldAng;
    private boolean m_LastDriveSkyStone = false;
    private boolean m_OnRecognition = false;
    private int recognitionResult = 0;
    @Override
    public Robot4100Generation1_LindaCore getRobotCore() {
        return m_RobotCore;
    }

    @Override
    public void hardwareInitialize() {
        this.m_RobotCore = new Robot4100Generation1_LindaCore(this.hardwareMap);
        RobotCamera Camera = new RobotOnPhoneCamera(this,false, RobotOnPhoneCamera.PhoneCameraDirection.Back, Robot4100Common.VUFORIA_LICENSE);
        this.m_SkyStoneDetection = new SkyStoneStoneDifferentiation(Camera,this.hardwareMap,Robot4100Generation1_Settings.AUTONOMOUS_TENSORFLOW_PREVIEW, Robot4100Generation1_Settings.AUTONOMOUS_MINIMUM_CONFIDENCE);
    }

    @Override
    public void hardwareDestroy() {
        this.m_RobotCore.terminate();
        this.m_SkyStoneDetection.terminate();
    }

    public void preRecAng(){
        this.m_RobotCore.getGyro().updateStatus();
        this.m_OldAng = m_RobotCore.getGyro().getHeading();
    }

    public boolean fixAng(){
        this.getRobotCore().getGyro().updateStatus();
        float newAng = getRobotCore().getGyro().getHeading();
        float deltaAng = newAng - m_OldAng;
        deltaAng = XYPlaneCalculations.normalizeDeg(deltaAng);
        if(deltaAng >= 1) {
            this.getRobotCore().getChassis().replaceTask(
                    new GyroGuidedTurn(
                            this.getRobotCore().getChassis(),
                            this.getRobotCore().getGyro(),
                            -deltaAng,
                            0.05
                    )
            );
            return waitForDrive();
        }else{
            return this.opModeIsActive();
        }

    }

    @Override
    public boolean waitForDrive(){
        while(this.opModeIsActive() && this.getRobotCore().getChassis().isBusy()){
            this.m_RobotCore.updateStatus();
        }
        __tryRecognizeStoneOnce();
        return this.opModeIsActive();
    }

    public void __tryRecognizeStoneOnce(){
        if(m_OnRecognition) {
            ArrayList<SkyStoneStoneDifferentiation.RecognitionResult> recognitionResults = this.m_SkyStoneDetection.getUpdatedRecognitions();
            if (recognitionResults != null && (!recognitionResults.isEmpty())) {
                for (SkyStoneStoneDifferentiation.RecognitionResult RRI : recognitionResults) {
                    if (RRI.getStoneType() == SkyStoneStoneDifferentiation.StoneType.SKYSTONE && (RRI.getTop() < RRI.getImageHeight() / 2) && (RRI.getBottom() > RRI.getImageHeight() / 2)){
                        m_LastDriveSkyStone = true;
                        break;
                    }
                }
            }
        }
    }

    public void __secondTimeRecognizeStoneOnce(){
        if(m_OnRecognition) {
            ArrayList<SkyStoneStoneDifferentiation.RecognitionResult> recognitionResults = this.m_SkyStoneDetection.getUpdatedRecognitions();
            if (recognitionResults != null && (!recognitionResults.isEmpty())) {
                for (SkyStoneStoneDifferentiation.RecognitionResult RRI : recognitionResults) {
                    if (RRI.getStoneType() == SkyStoneStoneDifferentiation.StoneType.SKYSTONE){
                        m_LastDriveSkyStone = true;
                        break;
                    }
                }
            }
        }
    }

    public void waitForGamepadX(){
        while(this.opModeIsActive() && gamepad1.x){
            this.getRobotCore().updateStatus();
        }
    }

    public Object getRecognitionResult(){
        return Integer.toString(this.recognitionResult);
    }

    @Override
    public void RunThisOpMode() {
        waitForStart();
        if(!this.opModeIsActive()) {
            return;
        }
        telemetry.addData("recognitionResult",getRecognitionResult());
        //Drive 34 cm to the right
        //Each stone is 20cm Wide
        //Distance between Camera & AutoGrabber = 4.8 cm
        //wall to the surface of the stone 75 cm

        double LENGTH_OF_EACH_STONE = Robot4100Generation1_Settings.AUTONOMOUS_LENGTH_FOR_EACH_STONE;

        m_LastDriveSkyStone = false;
        m_OnRecognition = true;
        preRecAng();
        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                42,
                0.2
        ));
        if(!waitForDrive()){
            return;
        }
        waitForGamepadX();

        this.m_SkyStoneDetection.setActivated(true);
        this.getRobotCore().getChassis().replaceTask(
                this.getRobotCore().getChassis().getFixedXDistanceTask(
                        -5,
                        0.1
                )
        );
        this.getRobotCore().getChassis().addTask(
                this.getRobotCore().getChassis().getFixedXDistanceTask(
                        5,
                        0.1
                )
        );
        if (!waitForDrive()) {
            return;
        }

        if(!fixAng()){
            return;
        }

        double firstStepExtraForwardDistance = 0;
        int[] stepNum = {1, 2, 3};

        for(int i=0;i<3;i++) {
            boolean foundStone = false;
            if (m_LastDriveSkyStone || i==2){
                foundStone = true;
            }
            if(foundStone){
                this.recognitionResult = stepNum[i];
                break;
            }
            m_OnRecognition = false;
            firstStepExtraForwardDistance += LENGTH_OF_EACH_STONE;
            this.getRobotCore().getChassis().replaceTask(
                    this.getRobotCore().getChassis().getFixedZDistanceTask(
                            LENGTH_OF_EACH_STONE,
                            0.2
                    )
            );

            if(!waitForDrive()){
                return;
            }
            m_OnRecognition = true;

            if(i==0) {
                this.getRobotCore().getChassis().replaceTask(
                        this.getRobotCore().getChassis().getFixedXDistanceTask(
                                -5,
                                0.1
                        )
                );
                this.getRobotCore().getChassis().addTask(
                        this.getRobotCore().getChassis().getFixedXDistanceTask(
                                5,
                                0.1
                        )
                );
                if (!waitForDrive()) {
                    return;
                }
            }

            telemetry.update();
            waitForGamepadX();
        }

        telemetry.update();

        //Grab Stone;
        this.m_SkyStoneDetection.setActivated(false);
        this.m_OnRecognition = false;

        if(true){
            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                    3,
                    0.1
            ));
            if(!waitForDrive()){
                return;
            }
            waitForGamepadX();
        }

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                75-42+20,
                0.2
        ));
        if(!waitForDrive()){
            return;
        }
        waitForGamepadX();

        this.getRobotCore().setAutonomousDragStoneServoToDrag(true);
        sleep(400);
        waitForGamepadX();

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                -70,
                0.2
        ));
        if(!waitForDrive()){
            return;
        }
        waitForGamepadX();

        if(!fixAng()){
            return;
        }

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                -(105 + firstStepExtraForwardDistance),
                0.6
        ));
        if(!waitForDrive()){
            return;
        }

        waitForGamepadX();

        ElapsedTime dragServoTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.getRobotCore().setAutonomousDragStoneServoToDrag(false);
        dragServoTime.reset();
        if(!fixAng()){
            return;
        }
        while(dragServoTime.seconds() < 0.300 && this.opModeIsActive()){

        }

        waitForGamepadX();


        if(recognitionResult != 3) {
            if(!fixAng()){
                return;
            }

            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                    (105 + ((2+3) *LENGTH_OF_EACH_STONE)),
                    0.6
            ));
            if (!waitForDrive()) {
                return;
            }


            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                    60,
                    0.3
            ));
            if (!waitForDrive()) {
                return;
            }
            waitForGamepadX();


            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                    -10,
                    0.3
            ));
            if (!waitForDrive()) {
                return;
            }
            waitForGamepadX();

            if(!fixAng()){
                return;
            }

            double secondStepExtraDistance = 10;
            if(recognitionResult == 1){
                secondStepExtraDistance = 0;
                this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                        -LENGTH_OF_EACH_STONE,
                        0.2
                ));
                if (!waitForDrive()) {
                    return;
                }
                waitForGamepadX();
            }else { //recognitionResult == 2){
                secondStepExtraDistance += LENGTH_OF_EACH_STONE;
            }

            if(!fixAng()){
                return;
            }

            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                    50,
                    0.2
            ));
            if (!waitForDrive()) {
                return;
            }
            waitForGamepadX();

            this.getRobotCore().setAutonomousDragStoneServoToDrag(true);
            sleep(400);
            waitForGamepadX();

            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                    -110,
                    0.2
            ));
            if(!waitForDrive()){
                return;
            }
            waitForGamepadX();

            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                    10,
                    0.2
            ));
            if(!waitForDrive()){
                return;
            }
            waitForGamepadX();

            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                    -(100 + ((1+3)*LENGTH_OF_EACH_STONE) + secondStepExtraDistance),
                    0.6
            ));
            if(!waitForDrive()){
                return;
            }

            waitForGamepadX();


            this.getRobotCore().setAutonomousDragStoneServoToDrag(false);
            sleep(300);
            waitForGamepadX();
        }

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                40,
                0.5
        ));
        if(!waitForDrive()){
            return;
        }

        waitForGamepadX();
    }
}
