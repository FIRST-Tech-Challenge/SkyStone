package org.darbots.darbotsftclib.testcases.OmniDriveTest;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.chassiscontrollers.OmniDrive;

@TeleOp(group = "DarbotsLib-TestCases", name = "OmniTest")
public class OmniTeleOpDarbots extends DarbotsBasicOpMode<OmniCore> {
    private OmniCore m_RobotCore;
    @Override
    public OmniCore getRobotCore() {
        return m_RobotCore;
    }

    @Override
    public void hardwareInitialize() {
        this.m_RobotCore = new OmniCore(this.hardwareMap);
        m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getTeleOpTask());
    }

    @Override
    public void hardwareDestroy() {
        this.m_RobotCore = null;
    }

    @Override
    public void RunThisOpMode() {
        OmniDrive.TeleOpControlTask TeleOpTask = (OmniDrive.TeleOpControlTask) m_RobotCore.getChassis().getCurrentTask();
        while(this.opModeIsActive()){
            TeleOpTask.setDriveRotationSpeed(-gamepad1.right_stick_x);
            TeleOpTask.setDriveXSpeed(gamepad1.left_stick_x);
            TeleOpTask.setDriveZSpeed(-gamepad1.left_stick_y);
            this.getRobotCore().updateStatus();
        }
    }
}
