package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;


    @TeleOp(name="Autonomous Options", group="Main")

    public class AutonomousOptions extends OpMode {

        // ADD preference names here
        public static final String START_POS_MODES_PREF = "starting position";
        public static final String DELAY_PREF = "delay";
        public static final String  PARKING_PREF = "parking";
        private static final String NONE = "none";
       // ADD preference values here
        private static final String[] START_POS_MODES = {"BLUE_2", "BLUE_3", "RED_2", "RED_3"};
        private static final String[] DELAYS = {"0 sec", "1 sec", "2 sec", "3 sec", "4 sec", "5 sec"};
        private static final String[] PARKING_LOCATION = {"underBridge", "buildingZone"};
        private static Map<String, String[]> prefMap = new HashMap<>();
        private static String[] prefKeys = prefMap.keySet().toArray(new String[prefMap.keySet().size()]);
        private static int keyIdx = 0;

        static {
            // ADD entries to preference map here
            prefMap.put(DELAY_PREF, DELAYS);
            prefMap.put(START_POS_MODES_PREF, START_POS_MODES);
            prefMap.put(PARKING_PREF, PARKING_LOCATION);
        }

        static {
            Arrays.sort(prefKeys);
        }

        public boolean isXPressed;
        public boolean isYPressed;
        private State menuState = State.DisplayAll;
        private SharedPreferences prefs;
        private SharedPreferences.Editor editor;
        private int selectionIdx = 0;

        public static SharedPreferences getSharedPrefs(HardwareMap hardwareMap) {
            return hardwareMap.appContext.getSharedPreferences("autonomous", 0);
        }

        private int getIndex(String val, String[] array) {
            if(array!=null){
                for (int i = 0; i < array.length; i++) {
                    if (array[i].equals(val)) {
                        return i;
                    }
                }
            }
            return -1;
        }

        @Override
        public void init() {
            prefs = getSharedPrefs(hardwareMap);
            editor = prefs.edit();
            editor.apply();
            for (String key : prefs.getAll().keySet()) {
                telemetry.addData(key, prefs.getString(key, NONE));
            }
        }

        @Override
        public void loop() {
            switch (menuState) {
                case DisplayAll:
                    displayAll();
                    Logger.logFile("in loop with DisplayAll");
                    break;
                case DisplaySingle:
                    Logger.logFile("in loop with Display single");
                    displaySingle();
                    break;
            }
        }

        private void displayAll () {
            telemetry.clear();
            telemetry.addData("Choose", "X - accept | Y - change");
//            for (String key : prefKeys) {
//                telemetry.addData(key, prefs.getString(key, NONE));
//            }
            for (String key : prefs.getAll().keySet()) {
                telemetry.addData(key, prefs.getString(key, NONE));
            }
            if (gamepad1.y && !isYPressed) {
                isYPressed = true;
                Logger.logFile("in display all, y is pressed, try to display single ");
                menuState = State.DisplaySingle;
            }
            if (!gamepad1.y) {
                isYPressed = false;
            }
        }

        private void displaySingle () {
            telemetry.clear();
            telemetry.addData("Choose", "X - accept Y - change");
            String key = prefKeys[keyIdx];
            Logger.logFile("here key = " + key);
            String[] array = prefMap.get(key);
            Logger.logFile("here, value[0] ="+array[0]);

            if (key != null) {
                String prefValue = prefs.getString(key, NONE);
                selectionIdx = getIndex(prefValue, array);
                telemetry.addData(key, "***" + prefValue);
                updateTelemetry(telemetry);
                Logger.logFile("key = ***" + prefValue);
                Logger.logFile("in display single, try to get key = " + key + " and prefValue = " + prefValue);
            }
            //accept and no change in current key, move to next key
            if (gamepad1.x && !isXPressed) {
                Logger.logFile("in display single, x is pressed, try to increase idx by 1. keyIdx = "+ keyIdx);
                Logger.logFile("gamepad1.x = "+gamepad1.x);
                int nextKeyIdx = keyIdx+1;
                Logger.logFile("now nextKeyIdx = " + nextKeyIdx);
                Logger.logFile("prefKeys.length = "+prefKeys.length);
                if (nextKeyIdx >= prefKeys.length && gamepad1.x) {
                    keyIdx = 0;
                    menuState = State.DisplayAll;
                }else {
                    keyIdx = nextKeyIdx;
                    Logger.logFile("I am surprise I am here but I suppose to be here");
                }
                Logger.logFile("I think keyIdx = " + keyIdx);
                isXPressed = true;
            }
            if (!gamepad1.x) {
                isXPressed = false;
            }

            //change to next idx in array
           if (gamepad1.y && !isYPressed) {
                Logger.logFile("in display single, y is pressed, try to move down selection list");
                Logger.logFile("gamepad1.y = "+ gamepad1.y);
                Logger.logFile("selectionIdx = " + selectionIdx);
                selectionIdx++;  //value[] idx
//                Logger.logFile(prefKeys[keyIdx+1]+" aaa");  //key idx
                if (selectionIdx >= array.length && gamepad1.y) {
                    selectionIdx = 0;
                }
                Logger.logFile("selectionIdx of key "+key +" is "+ selectionIdx);
                editor.putString(key, array[selectionIdx]);
                editor.apply();
                isYPressed = true;
            }
           if (!gamepad1.y) {
               isYPressed = false;
           }
        }

        enum State  {DisplayAll, DisplaySingle}
    }



