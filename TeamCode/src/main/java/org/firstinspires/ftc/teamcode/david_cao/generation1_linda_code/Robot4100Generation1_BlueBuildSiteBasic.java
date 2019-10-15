package org.firstinspires.ftc.teamcode.david_cao.generation1_linda_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.sensors.gyros.BNO055Gyro;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;

@Autonomous(name = "4100Gen1Auto-BlueBuildSiteBasic",group="4100")
public class Robot4100Generation1_BlueBuildSiteBasic extends DarbotsBasicOpMode<Robot4100Generation1_LindaCore> {
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
        return;
        /*
        this.m_RobotCore.getGyro().updateStatus();
        this.m_OldAng = m_RobotCore.getGyro().getHeading();
         */
    }

    public boolean fixAng(){
        return true;
        /*
        this.getRobotCore().getGyro().updateStatus();
        float newAng = getRobotCore().getGyro().getHeading();
        float deltaAng = newAng - m_OldAng;
        deltaAng = XYPlaneCalculations.normalizeDeg(deltaAng);
        this.getRobotCore().getChassis().replaceTask(
                this.getRobotCore().getChassis().getFixedTurnTask(
                        -deltaAng,
                        0.2
                )
        );
        return waitForDrive();
         */
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
                        -70,
                        0.4
                )
        );
        if(!waitForDrive()){
            return;
        }
        if(!fixAng()){
            return;
        }

        this.m_RobotCore.setDragServoToDrag(true);
        sleep(800);
        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        85,
                        0.4
                )
        );
        if(!waitForDrive()){
            return;
        }
        if(!fixAng()){
            return;
        }

        this.m_RobotCore.setDragServoToDrag(false);


        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        -3,
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
                        10,
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
                        -2,
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
                        -110,
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
                        -40,
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
                        35,
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
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        -55,
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
                        55,
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
                        -90,
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
        if(!fixAng()){
            return;
        }

        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        -50,
                        0.3
                )
        );
        if(!waitForDrive()){
            return;
        }
        if(!fixAng()){
            return;
        }

    }
}
