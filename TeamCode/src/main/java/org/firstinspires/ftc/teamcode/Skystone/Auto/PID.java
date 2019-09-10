package org.firstinspires.ftc.teamcode.Skystone.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto.TensorFlowMineralDetection;

@Autonomous(name="PID", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
public class PID extends AutoBase
{
    double lastx = 0;
    double integral = 0;

    public double control(int currentPos, int goal) {
        double p_gain = 0.4;
        double d_gain = 6.5;
        double i_gain = 0.001;

        double error = goal - currentPos;
        double deriv = currentPos - lastx;
        double integralNew = integral + error;

        lastx = currentPos;
        integral = integralNew;

        return (error * p_gain) + (integralNew * i_gain) - (deriv * d_gain);
    }

    @Override
    public void runOpMode(){
        //Init's robot
        initLogic();
        int goal = 1000;
        while(!gamepad1.x && opModeIsActive()){
            if(gamepad1.dpad_up){
                goal+=50;
            }else if(gamepad1.dpad_down){
                goal-=50;
            }
            sleep(2150);
            telemetry.addLine("Goal: " + goal);
            telemetry.update();
        }
        while (opModeIsActive()){
            //FIX THIS
            //robot.setDrivePower(control(robot.fLeft.getCurrentPosition(), goal));
            telemetry.addLine("fLeft : " +  control(robot.fLeft.getCurrentPosition(), goal));
            telemetry.update();
        }
    }
}

