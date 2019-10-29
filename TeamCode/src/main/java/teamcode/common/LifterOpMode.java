package teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Lifter test")
public class LifterOpMode extends TTOpMode{
    private DcMotor armLift;
    private Servo wrist, claw;
    private final double INCHES_TO_TICKS = 20;
    private int posistion = 1;
    //this is a constant that incriments each time y is clicked and is used to increment each time the robot is reset

    @Override
    protected void onInitialize() {
        armLift = hardwareMap.get(DcMotor.class, TTHardwareComponentNames.ARM_LIFT);
        wrist = hardwareMap.get(Servo.class, TTHardwareComponentNames.ARM_WRIST);
        claw = hardwareMap.get(Servo.class, TTHardwareComponentNames.ARM_CLAW);
    }

    @Override
    protected void onStart() {
        telemetry.addData("status","" );
        while(opModeIsActive()) {
            update();
        }
    }

    private void update(){
        telemetry.addData("status", "reached Update");
        telemetry.update();
        if(gamepad1.x){
            if(wrist.getPosition() == 0.0) {
                wrist.setPosition(1);
            }else{
                wrist.setPosition(0);
            }
        }else if(gamepad1.b){
            telemetry.addData("Status", "at b");
            telemetry.update();
            if(claw.getPosition() == 0.0) {
                claw.setPosition(0.7);
            }else{
                claw.setPosition(0.0);
            }

        }else if(gamepad1.a){
            scale(-5 * posistion);
        }else if(gamepad1.y){
            scale(5 * posistion);
        }
    }


    private void scale(int inches) {
        int ticks = (int)(inches * INCHES_TO_TICKS);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setTargetPosition(ticks);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(armLift.isBusy() && inches > 0){
            armLift.setPower(1);
        }
        while(armLift.isBusy() && inches < 0){
            armLift.setPower(-1);
        }
        armLift.setPower(0);
        if(inches > 0){
            posistion++;
        }
    }

    @Override
    protected void onStop() {

    }
    /* x, open/close
    b, swings servo 0, 0.7
    1500 ticks y
    a, resets to zero
    dpad clear
 */



}