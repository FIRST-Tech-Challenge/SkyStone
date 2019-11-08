package org.firstinspires.ftc.teamcode.david_cao.generation1_linda_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.GyroGuidedTurn;
import org.darbots.darbotsftclib.testcases.OmniDriveTest.OmniCore;

@TeleOp(group = "4100", name = "4100Gen1-MovementTesting")
public class Robot4100Generation1_MovementTest extends DarbotsBasicOpMode<Robot4100Generation1_LindaCore> {
    private Robot4100Generation1_LindaCore m_RobotCore;
    private String m_ChassisStatus;
    public String getChassisStatus(){
        return this.m_ChassisStatus;
    }
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
        this.m_RobotCore = null;
    }

    @Override
    public void RunThisOpMode() {
        waitForStart();
        if(!opModeIsActive()){
            return;
        }
        telemetry.addData("ChassisStatus",this.getChassisStatus());
        if(GlobalUtil.getGyro() != null) {
            this.m_RobotCore.getChassis().setGyroGuidedDriveEnabled(true);
            this.m_RobotCore.getChassis().updateGyroGuidedPublicStartingAngle();
        }
        
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
            }else if(this.gamepad1.dpad_left){
                this.m_RobotCore.getChassis().replaceTask(new GyroGuidedTurn(this.m_RobotCore.getChassis(),this.m_RobotCore.getGyro(),90,0.2));
            }else if(this.gamepad1.dpad_right){
                this.m_RobotCore.getChassis().replaceTask(new GyroGuidedTurn(this.m_RobotCore.getChassis(),this.m_RobotCore.getGyro(),-90,0.2));
            }
            telemetry.update();
        }
    }
}
