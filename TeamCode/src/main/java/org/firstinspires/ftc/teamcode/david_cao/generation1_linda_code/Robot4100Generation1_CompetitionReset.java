package org.firstinspires.ftc.teamcode.david_cao.generation1_linda_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;

@TeleOp(name = "4100Gen1-Tester-dcao",group="4100")
@Disabled
public class Robot4100Generation1_CompetitionReset extends DarbotsBasicOpMode {
    private Robot4100Generation1_LindaCore m_RobotCore;
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


    @Override
    public void RunThisOpMode() {
        waitForStart();
        if(this.opModeIsActive()){
            this.m_RobotCore.getLinearSlide().replaceTask(new TargetPosTask(null,getRobotCore().getLinearSlide().getMaxPos(),0.2));
            while(this.opModeIsActive() && this.m_RobotCore.getLinearSlide().isBusy() && !gamepad1.x){
                this.m_RobotCore.getLinearSlide().updateStatus();
            }
            this.m_RobotCore.getLinearSlide().deleteAllTasks();
        }
    }
}
