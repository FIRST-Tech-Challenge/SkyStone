package org.firstinspires.ftc.teamcode.roboticslibrary;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.opmodesupport.Joystick;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;

/**
 * Created by FIXIT on 16-10-24.
 */
public class CopyCatExecutor {

    private Gamepad[] actions = null;
    private long[] timing = null;
    private String fileName = "";

    public CopyCatExecutor(String fileName) {

        this.fileName = fileName;

        DataReader read = null;
        try {
            read = new DataReader(fileName);
        } catch (IOException e) {
            e.printStackTrace();
        }//catch

        String[] objects = read.readLines();

        actions = new Gamepad[objects.length];
        timing = new long[objects.length];

        for (int i = 0; i < objects.length; i++) {
            JSONObject obj = null;
            Gamepad gamepad = new Gamepad();

            try {
                obj = new JSONObject(objects[i]);

                gamepad.left_bumper = obj.getBoolean("lB");
                gamepad.right_bumper = obj.getBoolean("rB");
                gamepad.left_trigger = obj.getBoolean("lT")? 1 : 0;
                gamepad.right_trigger = obj.getBoolean("rT")? 1 : 0;

                gamepad.dpad_up = obj.getBoolean("up");
                gamepad.dpad_down = obj.getBoolean("do");
                gamepad.dpad_left = obj.getBoolean("le");
                gamepad.dpad_right = obj.getBoolean("ri");

                gamepad.a = obj.getBoolean("a");
                gamepad.b = obj.getBoolean("b");
                gamepad.x = obj.getBoolean("x");
                gamepad.y = obj.getBoolean("y");

                gamepad.start = obj.getBoolean("st");
                gamepad.back = obj.getBoolean("ba");

                gamepad.left_stick_y = (float) obj.getDouble("y1");
                gamepad.left_stick_x = (float) obj.getDouble("x1");
                gamepad.right_stick_y = (float) obj.getDouble("y2");
                gamepad.right_stick_x = (float) obj.getDouble("x2");

                timing[i] = obj.getLong("Time");

            } catch (JSONException e) {
                e.printStackTrace();
            }//catch

            actions[i] = gamepad;

        }//for

    }//CopyCatExecutor

    //runs on separate thread
    public void execute(final Joystick joy) {
        Runnable execute = new Runnable() {
            @Override
            public void run() {
                for (int i = 0; i < actions.length; i++) {
                    AutoOpMode.delay((int) timing[i]);
                    joy.update(actions[i]);
                }//for
            }
        };

        TaskHandler.addTask("CopyCatExecutor." + fileName.toUpperCase(), execute);
    }//execute

}
