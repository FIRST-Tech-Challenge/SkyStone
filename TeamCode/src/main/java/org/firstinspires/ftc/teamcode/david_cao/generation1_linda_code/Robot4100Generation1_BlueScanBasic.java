package org.firstinspires.ftc.teamcode.david_cao.generation1_linda_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.GyroGuidedTurn;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotCamera;
import org.darbots.darbotsftclib.season_specific.skystone.tfod_detection.SkyStoneStoneDifferentiation;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.util.ArrayList;

@Autonomous(name = "4100Gen1Auto-BlueScanBasic",group="4100")
public class Robot4100Generation1_BlueScanBasic extends DarbotsBasicOpMode {
    private SkyStoneStoneDifferentiation m_SkyStoneDetection;
    private Robot4100Generation1_LindaCore m_RobotCore;
    private float m_OldAng;
    private ArrayList<SkyStoneStoneDifferentiation.RecognitionResult> m_RecognitionResult = null;
    private int recognitionResult = 0;
    @Override
    public Robot4100Generation1_LindaCore getRobotCore() {
        return m_RobotCore;
    }

    @Override
    public void hardwareInitialize() {
        this.m_RobotCore = new Robot4100Generation1_LindaCore(this.hardwareMap);
        RobotCamera Camera = new RobotOnPhoneCamera(this,false, RobotOnPhoneCamera.PhoneCameraDirection.Back, Robot4100Common.VUFORIA_LICENSE);
        this.m_SkyStoneDetection = new SkyStoneStoneDifferentiation(Camera,this.hardwareMap,true, 0.5);
        this.m_SkyStoneDetection.setActivated(true);
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
        this.getRobotCore().getChassis().replaceTask(
                new GyroGuidedTurn(
                        this.getRobotCore().getChassis(),
                        this.getRobotCore().getGyro(),
                        -deltaAng,
                        0.05
                )
        );
        return waitForDrive();

    }

    @Override
    public boolean waitForDrive(){
        while(this.opModeIsActive() && this.getRobotCore().getChassis().isBusy()){
            this.m_RobotCore.updateStatus();
        }
        ArrayList<SkyStoneStoneDifferentiation.RecognitionResult> recognitionResults = this.m_SkyStoneDetection.getUpdatedRecognitions();
        if(recognitionResults != null && (!recognitionResults.isEmpty())){
            this.m_RecognitionResult = recognitionResults;
        }
        return this.opModeIsActive();
    }

    public void waitForGamepadX(){
        while(this.opModeIsActive() && !gamepad1.x){
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
        preRecAng();
        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                34,
                0.2
        ));
        if(!waitForDrive()){
            return;
        }
        waitForGamepadX();


        double firstStepExtraForwardDistance = 0;
        int step = 0;
        int[] stepNum = {1, 2, 3};

        for(int i=0;i<3;i++) {
            boolean foundStone = false;
            if (this.m_RecognitionResult != null && (!this.m_RecognitionResult.isEmpty())) {
                for (SkyStoneStoneDifferentiation.RecognitionResult RRI : this.m_RecognitionResult) {
                    if (RRI.getStoneType() == SkyStoneStoneDifferentiation.StoneType.SKYSTONE && RRI.getLeft() < (RRI.getImageWidth() / 2) && RRI.getRight() > (RRI.getImageWidth() / 2)){
                        foundStone = true;
                        break;
                    }
                }
            }
            if(i==2){
                foundStone = true;
            }
            if(foundStone){
                this.recognitionResult = stepNum[i];
                break;
            }
            this.getRobotCore().getChassis().replaceTask(
                    this.getRobotCore().getChassis().getFixedZDistanceTask(
                            -20,
                            0.2
                    )
            );
            firstStepExtraForwardDistance += 20;
            if(!waitForDrive()){
                return;
            }
            telemetry.update();
            waitForGamepadX();
        }

        telemetry.update();

        //Grab Stone;
        this.m_SkyStoneDetection.setActivated(false);
        this.m_RecognitionResult = null;

        if(true){ //(recognitionResult != 2){
            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                    3.5,
                    0.05
            ));
            if(!waitForDrive()){
                return;
            }
            waitForGamepadX();
        }

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                75-34+20,
                0.2
        ));
        if(!waitForDrive()){
            return;
        }
        waitForGamepadX();

        this.getRobotCore().setAutonomousDragStoneServoToDrag(true);
        sleep(500);
        waitForGamepadX();

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                -50,
                0.2
        ));
        if(!waitForDrive()){
            return;
        }
        waitForGamepadX();

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                130 + firstStepExtraForwardDistance,
                0.4
        ));
        if(!waitForDrive()){
            return;
        }

        waitForGamepadX();


        this.getRobotCore().setAutonomousDragStoneServoToDrag(false);
        sleep(500);
        waitForGamepadX();

        preRecAng();
        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                -75,
                0.2
        ));
        if(!waitForDrive()){
            return;
        }
        waitForGamepadX();

        preRecAng();

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                10,
                0.2
        ));

        if(!waitForDrive()){
            return;
        }
        waitForGamepadX();

        if(recognitionResult != 3){
            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                    -(130+firstStepExtraForwardDistance),
                    0.4
            ));
            if(!waitForDrive()){
                return;
            }

            if(!fixAng()){
                return;
            }

            waitForGamepadX();
            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                    -(3*20 + 5),
                    0.2
            ));
            if(!waitForDrive()){
                return;
            }
            waitForGamepadX();
            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                    90,
                    0.2
            ));
            if(!waitForDrive()){
                return;
            }
            waitForGamepadX();
            this.getRobotCore().setAutonomousDragStoneServoToDrag(true);
            sleep(500);
            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                    -140,
                    0.2
            ));
            if(!waitForDrive()){
                return;
            }

            if(!fixAng()){
                return;
            }

            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                    (140+firstStepExtraForwardDistance) + (3*20 + 5),
                    0.4
            ));
            if(!waitForDrive()){
                return;
            }
            waitForGamepadX();

            this.getRobotCore().setAutonomousDragStoneServoToDrag(false);
            sleep(500);

            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                    -10,
                    0.3
            ));
            if(!waitForDrive()){
                return;
            }
            waitForGamepadX();
        }


        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                -30,
                0.3
        ));
        if(!waitForDrive()){
            return;
        }
        waitForGamepadX();

    }
}
