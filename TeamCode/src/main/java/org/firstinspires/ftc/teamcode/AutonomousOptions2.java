package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;


@TeleOp(name="Autonomous Options2", group="Main")

public class AutonomousOptions2 extends OpMode {

    // ADD preference names here
    public static final String START_POS_MODES_PREF = "starting position";
    public static final String DELAY_PREF = "delay";
    public static final String  PARKING_PREF = "parking";
    private static final String NONE = "none";
    public static final String FOUNDATION_PREF = "foundation";
    public static final String DELIVER_ROUTE_PREF = "deliver mode";
    public static final String PARKING_ONLY_PREF = "only park";
    public static final String STONE_PREF = "skystones";
    public static final String FIRST_BLOCK_BY_WALL_PREF = "first block by wall";
    private static String[] prefKeys = {START_POS_MODES_PREF,STONE_PREF, DELAY_PREF, DELIVER_ROUTE_PREF,FOUNDATION_PREF,PARKING_ONLY_PREF,
            PARKING_PREF};
    // ADD preference values here
    public static final String[] START_POS_MODES = {"BLUE_2", "BLUE_3", "BLUE_5", "RED_2", "RED_3", "RED_5"};
    public static final String[] DELAYS = {"0 " + "sec", "1 sec", "2 sec", "3 sec", "4 sec", "5 sec"};
    public static final String[] PARKING_LOCATION = {"BridgeWall", "BridgeNeutral"};
    public static final String[] MOVE_FOUNDATION = {"move", "no move", "move only"};
    public static final String[] DELIVER_ROUTE = {"BridgeWall", "BridgeNeutral"};
    public static final String[] PARKING_ONLY = {"yes", "no"};
    public static final String[] STONE_OPTIONS = {"no", "group1","group2", "both"};
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
        prefMap.put(STONE_PREF, STONE_OPTIONS);
        prefMap.put(FIRST_BLOCK_BY_WALL_PREF, PICK_FIRST_BLOCK_BY_WALL);
    }

    //private static String[] prefKeys = prefMap.keySet().toArray(new String[prefMap.keySet().size()]);

    private static int keyIdx = 0;

//    static {
//        Arrays.sort(prefKeys);
//    }

    public boolean isUpPressed;
    public boolean isDownPressed;
    public boolean isRightPressed;
    public boolean isLeftPressed;
    private SharedPreferences prefs;
    private SharedPreferences.Editor editor;
    private int selectionIdx = 0;

    public static SharedPreferences getSharedPrefs(HardwareMap hardwareMap) {
        return hardwareMap.appContext.getSharedPreferences("autonomous", 0);
    }

    private int getIndex(String val, String[] array) {
        if (array!=null) {
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
        for (int i = 0; i < prefKeys.length; i++ ) {
            telemetry.addData(prefKeys[i], prefs.getString(prefKeys[i], NONE));
        }
    }

    @Override
    public void loop() {
        displayAll();
    }

    private void displayAll () {
        telemetry.clear();
//        telemetry.addData("Choose", "X - accept | Y - change");
//        for (String key : prefs.getAll().keySet()) {
//            telemetry.addData(key, prefs.getString(key, NONE));
//            if(keyIdx == )
//        }

        for (int i = 0; i < prefKeys.length; i++ ) {

            if (keyIdx == i) {
                String cap = prefKeys[i].toUpperCase();
                telemetry.addData("*" + cap + "*", prefs.getString(prefKeys[i], NONE));
            } else {
                telemetry.addData(prefKeys[i], prefs.getString(prefKeys[i], NONE));
            }
        }

        String key = prefKeys[keyIdx];
        String[] array = prefMap.get(key);

        if (key != null) {
            String prefValue = prefs.getString(key, NONE);
            selectionIdx = getIndex(prefValue, array);
            updateTelemetry(telemetry);
        }
        //accept and no change in current key, move to next key
        if (gamepad1.dpad_down && !isDownPressed) {
            int nextKeyIdx = keyIdx+1;

            if (nextKeyIdx >= prefKeys.length && gamepad1.dpad_down) {
                keyIdx = 0;
            } else {
                keyIdx = nextKeyIdx;
            }

            isDownPressed = true;
        }

        if (!gamepad1.dpad_down) {
            isDownPressed = false;
        }

        if (gamepad1.dpad_up && !isUpPressed) {
            int nextKeyIdx = keyIdx - 1;

            if (nextKeyIdx <= 0 && gamepad1.dpad_up) {
                keyIdx = prefKeys.length - 1;
            } else {
                keyIdx = nextKeyIdx;
            }

            isUpPressed = true;
        }

        if (!gamepad1.dpad_up) {
            isUpPressed = false;
        }

        //change to next idx in array
        if (gamepad1.dpad_right && !isRightPressed) {
            selectionIdx++;  //value[] idx

            if (selectionIdx >= array.length && gamepad1.dpad_right) {
                selectionIdx = 0;
            }

            editor.putString(key, array[selectionIdx]);
            updateAutoPref(key, array[selectionIdx]);
            editor.apply();
            isRightPressed = true;
        }

        if (!gamepad1.dpad_right) {
            isRightPressed = false;
        }

        if (gamepad1.dpad_left && !isLeftPressed) {
            selectionIdx--;  //value[] idx

            if (selectionIdx <= 0 && gamepad1.dpad_left) {
                selectionIdx = array.length-1;
            }

            editor.putString(key, array[selectionIdx]);
            updateAutoPref(key, array[selectionIdx]);
            editor.apply();
            isLeftPressed = true;
        }

        if (!gamepad1.dpad_left) {
            isLeftPressed = false;
        }
//        if (gamepad1.y && !isYPressed) {
//            isYPressed = true;
//            menuState = State.DisplaySingle;
//        }
//        if (!gamepad1.y) {
//            isYPressed = false;
//        }
    }
    void updateAutoPref(String key, String value) {
        if (key.equals(START_POS_MODES_PREF)) {
            if (value.equals("RED_5") || value.equals("BLUE_5")) {
                editor.putString(STONE_PREF, "no");
            }
            if(value.equals("RED_3") || value.equals("BLUE_3")){
                editor.putString(STONE_PREF, "group2");
            }
            if (value.equals("RED_2") || value.equals("BLUE_2")) {
                editor.putString(STONE_PREF, "group1"); // default
            }
        }
        if(key.equals(PARKING_ONLY_PREF) && value.equals("yes")) {
            editor.putString(STONE_PREF, "no");
            editor.putString(FOUNDATION_PREF, "no move");
        }
        if(key.equals(FOUNDATION_PREF) && value.equals("move only")) {
            editor.putString(STONE_PREF, "no");
        }

    }
}



