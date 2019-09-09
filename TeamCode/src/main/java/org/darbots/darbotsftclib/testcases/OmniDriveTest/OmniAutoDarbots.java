package org.darbots.darbotsftclib.testcases.OmniDriveTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.chassiscontrollers.OmniDrive;

@TeleOp(group = "DarbotsLib-TestCases", name = "OmniAutoTest")
@Disabled
public class OmniAutoDarbots extends DarbotsBasicOpMode<OmniCore> {
    private OmniCore m_RobotCore;
    private String m_ChassisStatus;
    public String getChassisStatus(){
        return this.m_ChassisStatus;
    }
    @Override
    public OmniCore getRobotCore() {
        return m_RobotCore;
    }

    @Override
    public void hardwareInitialize() {
        this.m_RobotCore = new OmniCore(this.hardwareMap);
    }

    @Override
    public void hardwareDestroy() {
        this.m_RobotCore = null;
    }

    @Override
    public void RunThisOpMode() {
        telemetry.addData("ChassisStatus",this.getChassisStatus());
        while(this.opModeIsActive()){
            this.m_RobotCore.updateStatus();
            if(this.m_RobotCore.getChassis().isBusy()){
                this.m_ChassisStatus = "Busy";
                continue;
            }
            this.m_ChassisStatus = "Not Busy";
            if(this.gamepad1.left_stick_y < -0.1){
                //Forward
                this.m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getFixedZDistanceTask(20,0.2));
            }else if(this.gamepad1.left_stick_y > 0.1){
                this.m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getFixedZDistanceTask(-20,0.2));
            }else if(this.gamepad1.left_stick_x < -0.1){
                this.m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getFixedXDistanceTask(-20,0.2));
            }else if(this.gamepad1.left_stick_x > 0.1){
                this.m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getFixedXDistanceTask(20,0.2));
            }else if(this.gamepad1.right_stick_x < -0.1){
                this.m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getFixedTurnTask(90,0.2));
            }else if(this.gamepad1.right_stick_x > 0.1){
                this.m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getFixedTurnTask(-90,0.2));
            }
            telemetry.update();
        }
    }
}
