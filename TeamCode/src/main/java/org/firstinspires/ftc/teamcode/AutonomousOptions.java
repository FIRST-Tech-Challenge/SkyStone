package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;


    @TeleOp(name="Autonomous Options", group="Main")
    @Disabled
    public class AutonomousOptions extends OpMode {

        // ADD preference names here
        public static final String START_POS_MODES_PREF = "starting position";
        public static final String DELAY_PREF = "delay";
        public static final String  PARKING_PREF = "parking";
        private static final String NONE = "none";
        public static final String FOUNDATION_PREF = "foundation";
        public static final String DELIVER_ROUTE_PREF = "deliver mode";
        public static final String PARKING_ONLY_PREF = "only park";
        public static final String STONE_PREF = "two skystones";
        public static final String FIRST_BLOCK_BY_WALL_PREF = "first block by wall";

        // ADD preference values here
        public static final String[] START_POS_MODES = {"BLUE_2", "BLUE_3", "BLUE_5", "RED_2", "RED_3", "RED_5"};
        public static final String[] DELAYS = {"0 " + "sec", "1 sec", "2 sec", "3 sec", "4 sec", "5 sec"};
        public static final String[] PARKING_LOCATION = {"BridgeWall", "BridgeNeutral"};
        public static final String[] MOVE_FOUNDATION = {"move", "no move", "move only"};
        public static final String[] DELIVER_ROUTE = {"BridgeWall", "BridgeNeutral"};
        public static final String[] PARKING_ONLY = {"yes", "no"};
        public static final String[] TWO_SKYSTONES = {"yes", "no"};
        public static final String[] PICK_FIRST_BLOCK_BY_WALL ={"yes", "no"};

        public static Map<String, String[]> prefMap = new HashMap<>();
        static {
            // ADD entries to preference map here
            prefMap.put(DELAY_PREF, DELAYS);
            prefMap.put(START_POS_MODES_PREF, START_POS_MODES);
            prefMap.put(PARKING_PREF, PARKING_LOCATION);
            prefMap.put(FOUNDATION_PREF, MOVE_FOUNDATION);
            prefMap.put(DELIVER_ROUTE_PREF, DELIVER_ROUTE);
            prefMap.put(PARKING_ONLY_PREF, PARKING_ONLY);
            prefMap.put(STONE_PREF, TWO_SKYSTONES);
            prefMap.put(FIRST_BLOCK_BY_WALL_PREF, PICK_FIRST_BLOCK_BY_WALL);

        }
        private static String[] prefKeys = prefMap.keySet().toArray(new String[prefMap.keySet().size()]);
        private static int keyIdx = 0;


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
                    break;
                case DisplaySingle:
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
            String[] array = prefMap.get(key);

            if (key != null) {
                String prefValue = prefs.getString(key, NONE);
                selectionIdx = getIndex(prefValue, array);
                telemetry.addData(key, "***" + prefValue);
                updateTelemetry(telemetry);
            }
            //accept and no change in current key, move to next key
            if (gamepad1.x && !isXPressed) {
                int nextKeyIdx = keyIdx+1;
                if (nextKeyIdx >= prefKeys.length && gamepad1.x) {
                    keyIdx = 0;
                    menuState = State.DisplayAll;
                }else {
                    keyIdx = nextKeyIdx;
                }
                isXPressed = true;
            }
            if (!gamepad1.x) {
                isXPressed = false;
            }

            //change to next idx in array
           if (gamepad1.y && !isYPressed) {
                selectionIdx++;  //value[] idx
                if (selectionIdx >= array.length && gamepad1.y) {
                    selectionIdx = 0;
                }
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



