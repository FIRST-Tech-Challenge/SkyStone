//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
//import com.qualcomm.robotcore.hardware.DigitalChannelController;
//
//import android.app.Activity;
//import android.graphics.Color;
//import android.hardware.Sensor;
//import android.view.View;
//
//import com.qualcomm.ftcrobotcontroller.R;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
//import com.qualcomm.robotcore.hardware.DigitalChannelController;
//import com.qualcomm.robotcore.util.RobotLog;
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "DiddlyDiddle", group = "Autonomous")
//
///**
// * Created by Instructor on 1/18/2018.
// */
//
//
//public class ProbablyDeleteLater extends LinearOpMode {
//    ColorSensor SensorRGB;
//    //DeviceInterfaceModule cdim;
//
//    static final int LED_CHANNEL = 5;
//
//    @Override
//    public void runOpMode() throws InterruptedException{
//        hardwareMap.logDevices();
//
//        //cdim = hardwareMap.deviceInterfaceModule.get("dim");
//
//        //cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
//
//        SensorRGB = hardwareMap.colorSensor.get("colour");
//
//        boolean bEnabled = true;
//
//        //cdim.setDigitalChannelState(LED_CHANNEL, bEnabled);
//
//        //wait one cycle
//
//        waitOneFullHardwareCycle();
//
//        waitForStart();
//
//        float
//                hsvValues[] = {
//                0F
//                ,
//                0F
//                ,
//                0F
//        };
//
//        final float
//                values[] = hsvValues;
//
//
//        final
//        View relativeLayout = ((Activity)
//                hardwareMap
//                        .
//                        appContext
//        ).findViewById(R.id.
//                RelativeLayout
//        );
//
//        boolean
//                bPrevState =
//                false
//                ;
//        boolean
//                bCurrState =
//                false
//                ;
//        while
//                (opModeIsActive()) {
//// check the status of the x button on either gamepad.
//            bCurrState =
//                    gamepad1
//                            .
//                            x
//                            ||
//                            gamepad2
//                                    .
//                                    x
//            ;
//// check for button state transitions.
//            if
//                    (bCurrState ==
//                    true
//                    && bCurrState !=
//                    bPrevState)  {
//// button is transitioning to a pressed state.
//// print a debug statement.
//                RobotLog.i
//                                (
//                                        "MY_DEBUG x button was pressed!"
//);
//// update previous state variable.
//                bPrevState = bCurrState;
//// on button press, enable the LED.
//                bEnabled =
//                        true
//                ;}
//// turn on the LED.
//
//               /*c
//                        .setDigitalChannelState(
//                                LED_CHANNEL
//                                , bEnabled);
//            }
//            else if
//                    (bCurrState ==
//                            false
//                            && bCurrState != bPrevState) {
//// button is transitioning to a released state.
//// print a debug statement.
//                RobotLog.i
//                                (
//                                        "MY_DEBUG x button was released!"
//                                );
//// update previous state variable.
//                bPrevState = bCurrState;
//// on button press, enable the LED.
//                bEnabled =
//                        false
//                ;
//// turn off the LED.
//                c
//                        .setDigitalChannelState(
//                                LED_CHANNEL
//                                , bEnabled);
//
//
//                Color.
//                        RGBToHSV
//                                ((       SensorRGB.red() * 255) / 800,
//                                        (SensorRGB.green() * 255) / 800,
//                                        (SensorRGB.blue() * 255) / 800, hsvValues);
//// send the info back to driver station using telemetry function.
//                telemetry.addData("Clear", SensorRGB.alpha());
//                telemetry.addData("Red  ", SensorRGB.red());
//                telemetry.addData("Green", SensorRGB.green());
//                telemetry.addData("Blue ", SensorRGB.blue());
//                telemetry.addData("Hue", hsvValues[0]);
//// change the background color to match the color detected by the RGB
//// pass a reference to the hue, saturation, and value array as an argument
//// to the HSVToColor method.
//        relativeLayout.post(new Runnable() {
//            public void run() {
//                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
//                                    }
//                                });
//// wait a hardware cycle before iterating.
//                waitOneFullHardwareCycle();
//            }
//        }
//    }
//
//}
//
//
//
//}*/}}}