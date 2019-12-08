package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="OmniR0", group="Linear Opmode")
//@Disabled

public class OmniR0 extends LinearOpMode {

  boolean isRed = true; //IMPORTANT

  private ElapsedTime runtime = new ElapsedTime();

  private DcMotor         driveNW = null;
  private DcMotor         driveNE = null;
  private DcMotor         driveSE = null;
  private DcMotor         driveSW = null;

  private DcMotor         slider  = null;
  private Servo           grabber = null;

  private ColorSensor     color   = null;

  private DistanceSensor  distN   = null;
  private DistanceSensor  distE   = null;
  private DistanceSensor  distS   = null;
  private DistanceSensor  distW   = null;

  double distN = 30;
  double distX = 0;
  double distS = 100;
  double distNCache = 0;

  double lumin = 1;

  double timeCache = 0;

  boolean isGrabbing = false;

  private void driveX(double force){

    // drive away from drivers

    if(isRed){
      force = -force;
    }

    driveNW.setPower( force);
    driveNE.setPower( force);
    driveSE.setPower(-force);
    driveSW.setPower(-force);

  }

  private void driveY(double force){

    // drive toward stones

    driveNW.setPower(-force);
    driveNE.setPower( force);
    driveSE.setPower( force);
    driveSW.setPower(-force);

  }

  private void driveSpn(double force){

    // spin clockwise red, cc blue

    if(isRed){
      force = -force;
    }

    driveNW.setPower( force);
    driveNE.setPower( force);
    driveSE.setPower( force);
    driveSW.setPower( force);

  }

  private void grab(boolean shouldGrab){

    if(shouldGrab){
      grabber.setPosition(0.7);
    }else{
      grabber.setPosition(0);
    }

  }

  private void senseUpdate(){
    distN = ( distN + senseDistN.getDistance(DistanceUnit.CM) ) / 2;
    distX = ( distX + senseDistX.getDistance(DistanceUnit.CM) ) / 2;
    distS = ( distS + senseDistS.getDistance(DistanceUnit.CM) ) / 2;
    lumin = ( lumin + (senseColor.red()+senseColor.green()+senseColor.blue()) / 3 ) / 2;
  }

  @Override
  public void runOpMode() {

    driveNW    = hardwareMap.get( DcMotor.class,       "driveNW" );
    driveNE    = hardwareMap.get( DcMotor.class,       "driveNE" );
    driveSE    = hardwareMap.get( DcMotor.class,       "driveSE" );
    driveSW    = hardwareMap.get( DcMotor.class,       "driveSW" );
    slider     = hardwareMap.get( DcMotor.class,       "slider"  );
    grabber    = hardwareMap.get( Servo.class  ,       "grabber" );
    senseColor = hardwareMap.get(ColorSensor.class,    "color"   );
    senseDistN = hardwareMap.get(DistanceSensor.class, "distN"   );
    senseDistX = hardwareMap.get(DistanceSensor.class, "distX"   );
    senseDistS = hardwareMap.get(DistanceSensor.class, "distS"   );

    waitForStart();
    runtime.reset();


    while(

      opModeIsActive() &&
      distN > 5 &&
      lumin < 0.3

    ){

      driveY(0.5);
      grab(false);

      senseUpdate();

    }


    distNCache = distN;

    while(

      opModeIsActive() &&
      distN - distNCache < 5

    ){

      driveY(-0.5);
      grab(false);

      senseUpdate();

    }


    while(

      opModeIsActive() &&
      distX < 50

    ){

      driveX(0.5);
      grab(false);

      senseUpdate();

    }


    while(

      opModeIsActive() &&
      distN - distNCache > 2

    ){

      driveY(0.5);

      senseUpdate();

    }


    grab(true);
    sleep(1000);


    while(

      opModeIsActive() &&
      distX > 50

    ){

      driveX(-0.5);

      senseUpdate();

    }


    while(

      opModeIsActive() &&
      distS > 100

    ){

      driveY(-0.5);

      senseUpdate();

    }


    driveX(0);

  }
}
