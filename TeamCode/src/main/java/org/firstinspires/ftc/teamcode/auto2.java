package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "auto2")
public class auto2 extends LinearOpMode {
    TypexChart chart = new TypexChart();
    CONSTANTS constants = new CONSTANTS();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        TypexChart chart = new TypexChart();
        chart.init(hardwareMap);

        waitForStart();

        runtime.reset();

        //strafeRight();
        chart.TL.setPower(constants.STRAFEPOWER);
        chart.TR.setPower(-constants.STRAFEPOWER);
        chart.BL.setPower(-constants.STRAFEPOWER);
        chart.BR.setPower(constants.STRAFEPOWER);
        while (opModeIsActive()&&runtime.seconds()<3){

        }

        runtime.reset();
        //strafeLeft();
        chart.TL.setPower(-constants.STRAFEPOWER);
        chart.BL.setPower(constants.STRAFEPOWER);
        chart.TR.setPower(constants.STRAFEPOWER);
        chart.BR.setPower(-constants.STRAFEPOWER);
        while (opModeIsActive()&&runtime.seconds()<3){

        }

        runtime.reset();
        //drive(0);
        chart.TL.setPower(-constants.STRAFEPOWER);
        chart.BL.setPower(constants.STRAFEPOWER);
        chart.TR.setPower(constants.STRAFEPOWER);
        chart.BR.setPower(-constants.STRAFEPOWER);
    }

    /*
    ------------------------------------------------------------------------------------
    METHODS USED FOR NAVIGATION
    ------------------------------------------------------------------------------------
     */


    public void drive(double power){
        chart.TL.setPower(power);
        chart.TR.setPower(power);
        chart.BL.setPower(power);
        chart.BR.setPower(power);
    }

    public void strafeRight() {
        chart.TL.setPower(constants.STRAFEPOWER);
        chart.TR.setPower(-constants.STRAFEPOWER);
        chart.BL.setPower(-constants.STRAFEPOWER);
        chart.BR.setPower(constants.STRAFEPOWER);
    }

    public void strafeLeft() {
        chart.TL.setPower(-constants.STRAFEPOWER);
        chart.BL.setPower(constants.STRAFEPOWER);
        chart.TR.setPower(constants.STRAFEPOWER);
        chart.BR.setPower(-constants.STRAFEPOWER);
    }

}
