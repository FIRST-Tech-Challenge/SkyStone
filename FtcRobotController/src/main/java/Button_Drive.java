import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Button_Drive")
public class Button_Drive extends OpMode{
    DcMotor lf, rf, lb, rb;
    public Gamepad g1, g2;
    private ElapsedTime runtime = new ElapsedTime();

   @Override
    public void init() {
        telemetry.addData("Status", "Initialized");



        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");

        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

       lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Runs based on speed instead of voltage; makes run more consistently
       rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       
    }

    @Override
    public void loop() {
       if (gamepad1.a){
           rf.setPower(1);
       }else{
           rf.setPower(0);
       }

       if (gamepad1.b){
           rb.setPower(1);
       }else{
           rb.setPower(0);
       }
       if (gamepad1.x){
           lf.setPower(1);
       }else{
           lf.setPower(0);
       }
       if (gamepad1.y){
           lb.setPower(1);
       }else{
           lb.setPower(0);
       }
        telemetry.addData("rb", rb.getCurrentPosition());
        telemetry.addData("rf", rf.getCurrentPosition());
        telemetry.addData("lf", lf.getCurrentPosition());
        telemetry.addData("lb", lb.getCurrentPosition());
        telemetry.update();
        if (gamepad1.right_bumper){
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

}
