package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class ArmMove {
    
    private DcMotor rearMotor;
    private DcMotor frontMotor;
    
    private HardwareMap hardwareMap;
    LinearOpMode opmode;
    public ArmMove(LinearOpMode opmode) {
    this.opmode = opmode;
    this.hardwareMap = opmode.hardwareMap;
    } 
    public void initHardware() {
    frontMotor = hardwareMap.dcMotor.get("frontMotor");
    rearMotor = hardwareMap.dcMotor.get("rearMotor");
    
    rearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    opmode.sleep(500);
    
    rearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    private ElapsedTime FPIDTime = new ElapsedTime();
    
    private ElapsedTime RPIDTime = new ElapsedTime();
    private ElapsedTime PIDTime = new ElapsedTime();
    
    private ElapsedTime Timeout = new ElapsedTime();
    
    private double RPrevError = 0;
    private double FPrevError = 0;
    private double RTotalError = 0;
    private double FTotalError = 0;
    private double RSpeed = 0;
    private double FSpeed = 0;
    private double FkP = 0.012; //0.012
    private double FkI = 0.001; //0.006
    private double FkD = 0.001;//0.0012
        
    private double RkP = 0.0085; //0.009
    private double RkI = 0.000; //0.006
    private double RkD = 0.0009;//0.0008
    
    private double MaxSpeedR = 0.8;
    private double MaxSpeedF = 0.5;

    private long MaxTimeOut = 1;
    
    private boolean EndConditionArmF = false;
    private boolean EndConditionArmR = false;
        public void ArmLoop(double TarPosR, double TarPosF, double MSR, double MSF) {
            RPrevError = 0;
            FPrevError = 0;
            RTotalError = 0;
            FTotalError = 0;
            RSpeed = 0;
            FSpeed = 0;
            EndConditionArmF = false;
            EndConditionArmR = false;
            Timeout.reset();
            
            FPIDTime.reset();
            RPIDTime.reset();
            Timeout.reset();
            
            while(Timeout.seconds() < MaxTimeOut) {
                PIDArm(rearMotor.getCurrentPosition(), TarPosR, RkP, RkI, RkD, 0, MSR);
                PIDArm(frontMotor.getCurrentPosition(), TarPosF, FkP, FkI, FkD, 1,MSF);
                
                rearMotor.setPower(RSpeed);
                frontMotor.setPower(FSpeed);
                
            }
        }

        public void UseTheForce() {
            rearMotor.setPower(-0.5);
        }



                                                     //0 is rearMotor 1 is frontMotor \/
        public void PIDArm(double EV, double TPos, double kP, double kI, double kD, int motor, double MaxSpeed) {
            
            double DError = 0;
            int DBanMin = -1;
            int DBanMax = 1;
            int MaxError = 10;
            double error = 0;
            double speed = 0;
            double TotalError = 0;
            double PrevError = 0;
            boolean EndConditionArm = false;
            if(motor == 0) {
                TotalError = RTotalError;
                PrevError = RPrevError;
                PIDTime = RPIDTime;
                EndConditionArm = EndConditionArmR;
            } else if(motor == 1){
                TotalError = FTotalError;
                PrevError = FPrevError;
                PIDTime = FPIDTime;
                EndConditionArm = EndConditionArmF;
            }
            
            
            
            
            //calculate error (Proportional)
            error = TPos - EV;
            
            //Calculate Total error (Integral)
            TotalError = (error * PIDTime.seconds()) + TotalError;
            if(error < 5 && error > -5) {EndConditionArm = true;} else {EndConditionArm = false;}
            //do deadband
            if(DBanMax > error && error > DBanMin) {
              error = 0;
              //TotalError = 0;
            }
            
            //calculate delta error (Derivative)
            DError = (error - PrevError) / PIDTime.seconds();
            
            //reset elapsed timer
            PIDTime.reset();
            
            //Max total error
            if(Math.abs(TotalError) > MaxError) {
                
            if(TotalError > 0) {
                    TotalError = MaxError;
                } else {
                    TotalError = -MaxError;
                }
                
            }
            
            
            //Calculate final speed
            speed = (error * kP) + (TotalError * kI) + (DError * kD);
            
            
            //Make sure speed is no larger than MaxSpeed
            if(Math.abs(speed) > MaxSpeed) {
                 if(speed > 0) {
                    speed = MaxSpeed;
                } else {
                    speed = -MaxSpeed;
                }
            }
            PrevError = error;
            
            if(motor == 0) {
             RSpeed = speed;
             RPrevError = PrevError;
             RTotalError = TotalError;
             EndConditionArmR = EndConditionArm;
            } else if(motor == 1) {
            FSpeed = speed;
            FPrevError = PrevError;
            FTotalError = TotalError;
            EndConditionArmF = EndConditionArm;
            }
            //set previous error to error
            
            
            //add telemetry
            
            
    
        }
        
}