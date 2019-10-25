package org.firstinspires.ftc.teamcode.david_cao.generation1_linda_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.GyroGuidedTurn;

@Autonomous(name = "4100Gen1Auto-RedBuildSiteBasic",group="4100")
public class Robot4100Generation1_RedBuildSiteBasic extends DarbotsBasicOpMode<Robot4100Generation1_LindaCore> {
    private Robot4100Generation1_LindaCore m_RobotCore;
    private float m_OldAng;
    @Override
    public Robot4100Generation1_LindaCore getRobotCore() {
        return m_RobotCore;
    }

    @Override
    public void hardwareInitialize() {
        this.m_RobotCore = new Robot4100Generation1_LindaCore(this.hardwareMap);
    }

    @Override
    public void hardwareDestroy() {
        this.m_RobotCore.terminate();
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

    public void waitForGamepadX(){
        while(this.opModeIsActive() && gamepad1.x){
            this.getRobotCore().updateStatus();
        }
    }


    @Override
    public void RunThisOpMode() {
        waitForStart();
        if(!this.opModeIsActive()) {
            return;
        }

        preRecAng();
        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        -40,
                        0.4
                )
        );
        if(!waitForDrive()){
            return;
        }

        if(!fixAng()){
            return;
        }

        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        -30,
                        0.4
                )
        );
        if(!waitForDrive()){
            return;
        }

        if(!fixAng()){
            return;
        }

        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        -35,
                        0.4
                )
        );
        if(!waitForDrive()){
            return;
        }

        this.m_RobotCore.setDragServoToDrag(true);
        sleep(500);
        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        95,
                        0.4
                )
        );
        if(!waitForDrive()){
            return;
        }

        this.m_RobotCore.setDragServoToDrag(false);
        sleep(300);

        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        95,
                        0.3
                )
        );
        if(!waitForDrive()){
            return;
        }


        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        -45,
                        0.2
                )
        );
        if(!waitForDrive()){
            return;
        }


        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        -45,
                        0.2
                )
        );

        if(!waitForDrive()){
            return;
        }


        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        10,
                        0.2
                )
        );
        if(!waitForDrive()){
            return;
        }


        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        -70,
                        0.3
                )
        );
        if(!waitForDrive()){
            return;
        }
        if(!fixAng()){
            return;
        }

        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        -80,
                        0.3
                )
        );
        if(!waitForDrive()){
            return;
        }


        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        55,
                        0.3
                )
        );
        if(!waitForDrive()){
            return;
        }


        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        -10,
                        0.2
                )
        );
        if(!waitForDrive()){
            return;
        }
        if(!fixAng()){
            return;
        }


        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        90,
                        0.3
                )
        );
        if(!waitForDrive()){
            return;
        }
        if(!fixAng()){
            return;
        }


        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        80,
                        0.3
                )
        );
        if(!waitForDrive()){
            return;
        }
        if(!fixAng()){
            return;
        }
        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        -5,
                        0.2
                )
        );
        if(!waitForDrive()){
            return;
        }
        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        50,
                        0.3
                )
        );
        if(!waitForDrive()){
            return;
        }
    }
}
