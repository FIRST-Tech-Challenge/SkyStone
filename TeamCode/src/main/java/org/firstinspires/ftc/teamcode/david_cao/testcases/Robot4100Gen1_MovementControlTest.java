package org.firstinspires.ftc.teamcode.david_cao.testcases;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.firstinspires.ftc.teamcode.david_cao.generation1_linda_code.Robot4100Generation1_LindaCore;

@TeleOp(name="4100Gen1_AutoMovementControl",group="4100")
public class Robot4100Gen1_MovementControlTest extends DarbotsBasicOpMode<Robot4100Generation1_LindaCore> {
    private Robot4100Generation1_LindaCore m_RobotCore;
    private static final double GAMEPAD_TRIGGER_VALUE = 0.2;
    @Override
    public Robot4100Generation1_LindaCore getRobotCore() {
        return m_RobotCore;
    }

    @Override
    public void hardwareInitialize() {
        m_RobotCore = new Robot4100Generation1_LindaCore(this.hardwareMap);
    }

    @Override
    public void hardwareDestroy() {
        m_RobotCore.terminate();
    }

    @Override
    public void RunThisOpMode() {
        waitForStart();
        while(this.opModeIsActive()){
            if(!this.m_RobotCore.getChassis().isBusy()){
                if(gamepad1.left_stick_x > GAMEPAD_TRIGGER_VALUE){
                    this.m_RobotCore.getChassis().replaceTask(
                            this.m_RobotCore.getChassis().getFixedXDistanceTask(
                                    20,
                                    0.2
                            )
                    );
                }else if(gamepad1.left_stick_x < -GAMEPAD_TRIGGER_VALUE){
                    this.m_RobotCore.getChassis().replaceTask(
                            this.m_RobotCore.getChassis().getFixedXDistanceTask(
                                    -20,
                                    0.2
                            )
                    );
                }else if((-gamepad1.left_stick_y) > GAMEPAD_TRIGGER_VALUE){
                    this.m_RobotCore.getChassis().replaceTask(
                            this.m_RobotCore.getChassis().getFixedZDistanceTask(
                                    20,
                                    0.2
                            )
                    );
                }else if((-gamepad1.left_stick_y) < -GAMEPAD_TRIGGER_VALUE){
                    this.m_RobotCore.getChassis().replaceTask(
                            this.m_RobotCore.getChassis().getFixedZDistanceTask(
                                    -20,
                                    0.2
                            )
                    );
                }else if(gamepad1.right_stick_x > GAMEPAD_TRIGGER_VALUE){
                    this.m_RobotCore.getChassis().replaceTask(
                            this.m_RobotCore.getChassis().getFixedTurnTask(
                                    -45,
                                    0.2
                            )
                    );
                }else if(gamepad1.right_stick_x < -GAMEPAD_TRIGGER_VALUE){
                    this.m_RobotCore.getChassis().replaceTask(
                            this.m_RobotCore.getChassis().getFixedTurnTask(
                                    45,
                                    0.2
                            )
                    );
                }
            }
            this.m_RobotCore.updateStatus();
        }
    }
}
