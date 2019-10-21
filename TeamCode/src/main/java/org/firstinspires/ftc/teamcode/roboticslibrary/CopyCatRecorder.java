package org.firstinspires.ftc.teamcode.roboticslibrary;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.Joystick;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.UUID;

/**
 * Created by FIXIT on 16-10-24.
 */
public class CopyCatRecorder {

    private ArrayList<Gamepad> actions = new ArrayList<>();
    private ArrayList<Long> timing = new ArrayList<>();
    private String fileName = "";
    private Runnable recordJoy;

    public CopyCatRecorder(String fileName, final Joystick joy) {
        this.fileName = fileName;
        actions.add(joy.gamepad);
        timing.add(System.currentTimeMillis());

        recordJoy = new Runnable() {
            @Override
            public void run() {
                if (!joy.gamepad.equals(actions.get(actions.size() - 1))) {
                    actions.add(joy.gamepad);
                    timing.add(System.currentTimeMillis());
                }//if
            }
        };
    }//CopyCat

    public CopyCatRecorder(Joystick joy) {
        this(UUID.randomUUID().toString(), joy);
    }//CopyCat

    public void recordJoystick() {
        TaskHandler.addLoopedTask("CopyCatRecorder." + fileName.toUpperCase(), recordJoy, 100);
    }//recordJoystick

    public void terminate() {
        TaskHandler.removeTask("CopyCat: " + fileName);
    }//terminate

    public void writeToFile() {

        RC.t.setDataLogFile(fileName, true);

        for (int i = 0; i < actions.size(); i++) {

            JSONObject obj = new JSONObject();
            Gamepad joy = actions.get(i);

            try {
                obj.put("a", joy.a);
                obj.put("b", joy.b);
                obj.put("x", joy.x);
                obj.put("y", joy.y);

                obj.put("ba", joy.back);
                obj.put("st", joy.start);

                obj.put("up", joy.dpad_up);
                obj.put("do", joy.dpad_down);
                obj.put("le", joy.dpad_left);
                obj.put("ri", joy.dpad_right);

                obj.put("lB", joy.left_bumper);
                obj.put("lT", joy.left_trigger > 0.1);
                obj.put("rB", joy.right_bumper);
                obj.put("rT", joy.right_trigger > 0.1);

                obj.put("y1", joy.left_stick_y);
                obj.put("x1", joy.left_stick_x);
                obj.put("y2", joy.right_stick_y);
                obj.put("x2", joy.right_stick_x);

                obj.put("Time", timing.get(i) - timing.get(0));
            } catch (JSONException e) {
                e.printStackTrace();
            }//catch

            RC.t.dataLogData(fileName, obj.toString());
        }//for

        RC.t.close();
    }//writeToFile

}
