package org.firstinspires.ftc.teamcode.opModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
@TeleOp(name = "TeleOpMode", group = "Teleop")
public class TeleOpMode extends LinearOpMode {
    public void runOpMode() {
        DcMotor hook = hardwareMap.dcMotor.get("front_left_motor");
        telemetry.addData("Init","v:1.0");
        waitForStart();
        
        while (opModeIsActive()) {
            double power;
            
            if (gamepad1.a == 1) {
                motor.setpower(1);
                
            }
            else if (gamepad1.b == 1) {
                motor.setpower(-1);
            
            }
            else {
                motor.setpower(0);
            }
            
            
            telemetry.addData("climb", hook.getCurrentPosition());
            telemetry.update();
        }
    }
}  
