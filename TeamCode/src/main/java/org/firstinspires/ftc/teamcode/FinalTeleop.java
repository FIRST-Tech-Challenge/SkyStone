/*
Copyright 2018 FIRST Tech Challenge Team [Phone] SAMSUNG

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.Func;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import java.util.Locale;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
//@Disabled
@TeleOp(name="Final_TeleOp", group="Linear Opmode")
public class FinalTeleop extends LinearOpMode {
    private BNO055IMU imu;
    private DcMotor frontRightMotor;
    private DcMotor rearRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor rearLeftMotor;
    private Blinker expansion_Hub_2;
    private DcMotor frontMotor;
    private DcMotor rearMotor;
    Orientation angles;
    Acceleration gravity;
    
    Orientation             lastAngles = new Orientation();
    
    double globalAngle = 0.0;
    
    
    private ElapsedTime FPIDTime = new ElapsedTime();
    
    private ElapsedTime RPIDTime = new ElapsedTime();
    
    private ElapsedTime ZPIDTime = new ElapsedTime();
    
    private ElapsedTime PIDTime = new ElapsedTime();
    
    private double RPrevError = 0;
    private double FPrevError = 0;
    private double ZPrevError = 0;
    
    private double RTotalError = 0;
    private double FTotalError = 0;
    private double ZTotalError = 0;
    
    private double RSpeed = 0;
    private double FSpeed = 0;
    private double ZSpeed = 0;
    
        
    private double FDError = 0;
    private double RDerror = 0;
    private double ZDerror = 0;
    
    private double FkP = 0.012; //0.012
    private double FkI = 0.001; //0.001
    private double FkD = 0.001;//0.001
        
    private double RkP = 0.0085; //0.0085
    private double RkI = 0.000; //0.000
    private double RkD = 0.0009;//0.0009
    
    private double ZkP = 0.0321; //0.0321
    private double ZkI = 0.000; //0.000
    private double ZkD = 0.00535;//0.00535

    
    private double REV = -14;
    private double FEV = 7;
    private double ZTar = 0;
    

    private double MaxSpeedR = 0.5;
    private double MaxSpeedF = 0.5;
    private double MaxSpeedZ = 1.0;



    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "Rear_Left_Motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "Rear_Right_Motor");
        frontMotor = hardwareMap.get(DcMotor.class, "frontMotor");
        rearMotor = hardwareMap.get(DcMotor.class, "rearMotor");
//        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 2");



        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        imu.initialize(parameters);

        resetAngle();
        
        
        
        
        rearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
        
        rearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        
        double LMP = 0;
        double RMP = 0;
        double FMP = 0;
        double BMP = 0;
        
        double FX = 0;
        double FY = 0;
        
        double CA = 0;
        
        double RSA = 0;
        
        double Mag = 0;
        double zMag = 0;
        
        double mult = 1; //THIS IS SPEED
        double multZ = 0.3; 
        
        double power = 1;
        double upMult = 1;
        
        double joyX = 0;
        double joyY = 0;
        
        //int targetR = -80; 
        //int targetF = 60;
        

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            
            sleep(50);
            idle();
        }
        
        waitForStart();
        
        FPIDTime.reset();
        RPIDTime.reset();
        ZPIDTime.reset();
        

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            joyX = gamepad2.left_stick_x;
            joyY = gamepad2.left_stick_y;
            
            if (gamepad1.a) {
                resetAngle();
                
                
            } 
            
            zMag = (joyX * joyX) + (joyY * joyY);
            
            
            if(Math.sqrt(zMag) > 0.5) {
                ZTar = Math.atan2(-joyX, -joyY)*180/3.14159;
                
            }
            
            
            CA = (Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) * 180 / 3.14) + 45;
            
            RSA = (CA - getAngle()) * 3.14 / 180;
            
            Mag = Math.sqrt((gamepad1.right_stick_y * gamepad1.right_stick_y) + (gamepad1.right_stick_x * gamepad1.right_stick_x));
            
            FX = -Math.sin(RSA) * Mag;
            FY = -Math.cos(RSA) * Mag;
            
            
            LMP = (ZSpeed * multZ) + FX; //multZ will only affect Z. This is because if joypad Z is zero then Z is zero.
            RMP = (ZSpeed * multZ) - FX;
            FMP = (ZSpeed * multZ) + FY;
            BMP = (ZSpeed * multZ) - FY;

            frontLeftMotor.setPower(RMP * mult);
            rearRightMotor.setPower(LMP * mult);

            rearLeftMotor.setPower(FMP * mult);
            frontRightMotor.setPower(BMP * mult);
            
            if(gamepad2.a){ //pick up
                REV = -135;
                FEV = 25;
                MaxSpeedR = 0.8; //0.8
                MaxSpeedF = 0.5; // 0.5
            } else if(gamepad2.b) { //grab
                REV = -135;
                FEV = 143;
                MaxSpeedR = 0.5; //0.5
                MaxSpeedF = 0.5; //0.5
            } else if(gamepad2.y) { //drop
                REV = -60; 
                FEV = 135;
                MaxSpeedR = 0.2; // 0.2
                MaxSpeedF = 0.6; //0.6
            } else if(gamepad2.x) { //home
                REV = -10;
                FEV = 10;
                MaxSpeedR = 0.8; // 0.6
                MaxSpeedF = 0.2; // 0.2
            } else if(gamepad2.right_bumper) { //drop 2
                REV = -50;
                FEV = 110;
                MaxSpeedR = 0.2; //.2
                MaxSpeedF = 0.6; //.6
            } else if(gamepad2.left_bumper) { //foundation
            rearMotor.setPower(0.5);
            frontMotor.setPower(0.5);
            }
            
            
            
            PIDArm(rearMotor.getCurrentPosition(), REV, RkP, RkI, RkD, 0);
            PIDArm(frontMotor.getCurrentPosition(), FEV, FkP, FkI, FkD, 1);
            
            PIDArm(getAngle(), ZTar, ZkP, ZkI, ZkD, 67);//CAN BE ANYTHING BUT 0 OR 1
            
            rearMotor.setPower(RSpeed);
            frontMotor.setPower(FSpeed);

                        //add telemetry
                        
            
            
            telemetry.addData("X", FX);
            telemetry.addData("Y", FY);
            telemetry.addData("CA", CA);
            telemetry.addData("RSA", RSA);
            telemetry.addData("RA", getAngle());

            telemetry.addData("Rear Encoder Value", rearMotor.getCurrentPosition());
            telemetry.addData("Front Encoder Value", frontMotor.getCurrentPosition());
            telemetry.addData("Front Total Error", FTotalError);
            telemetry.addData("Rear Total Error", RTotalError);
            telemetry.addData("Rear Previous Error", RPrevError);
            telemetry.addData("Front Previous Error", FPrevError);
            telemetry.addData("Rear Speed", RSpeed);
            telemetry.addData("Rear Speed", FSpeed);
            telemetry.addData("zMag", zMag);
            telemetry.addData("ZTar", ZTar);
            
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
    
                                                     //0 is rearMotor 1 is frontMotor \/
        public void PIDArm(double EV, double TPos, double kP, double kI, double kD, int motor) {
            
            double DError = 0;
            int DBanMin = -1;
            int DBanMax = 1;
            int MaxError = 10;
            double error = 0;
            double speed = 0;
            double TotalError = 0;
            double PrevError = 0;
            double MaxSpeed = 0;

            
            if(motor == 0) {
                TotalError = RTotalError;
                PrevError = RPrevError;
                PIDTime = RPIDTime;
                MaxSpeed = MaxSpeedR;
            } else if(motor == 1){
                TotalError = FTotalError;
                PrevError = FPrevError;
                PIDTime = FPIDTime;
                MaxSpeed = MaxSpeedF;
            } else {
                TotalError = ZTotalError;
                PrevError = ZPrevError;
                PIDTime = ZPIDTime;
                MaxSpeed = MaxSpeedZ;

            }
            
            
            
            
            //calculate error (Proportional)
            error = TPos - EV;
            
            //Calculate Total error (Integral)
            TotalError = (error * PIDTime.seconds()) + TotalError;
            
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
            } else if(motor == 1) {
            FSpeed = speed;
            FPrevError = PrevError;
            FTotalError = TotalError;
            } else {
            ZSpeed = speed;
            ZPrevError = PrevError;
            ZTotalError = TotalError;
            }
            //set previous error to error
            
            
            //add telemetry
            

            
    
        }
    
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    

    //Function to get the angle of the Gyro sensor
    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
                                
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}
