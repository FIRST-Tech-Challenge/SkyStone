package org.firstinspires.ftc.robotcontroller.internal;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.text.InputType;
import android.text.method.DigitsKeyListener;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.RelativeLayout;
import android.widget.Space;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.ToggleButton;

import com.qualcomm.ftcrobotcontroller.R;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

public class GlobalValuesActivity extends Activity {

    public static HashMap<String, Object> globals = new HashMap<>();
    public static ArrayList<String> autoKeys = new ArrayList<String>();

    public static String GLOBALS_FILE_PATH = "";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_global_values);

        RelativeLayout insert = (RelativeLayout) findViewById(R.id.globalsettings);

        if (!globals.isEmpty()) {

            int idCount = 0;

            for (Iterator<Map.Entry<String, Object>> globIter = globals.entrySet().iterator(); globIter.hasNext(); ) {

                if (idCount > 0) {

                    View line = new View(this);
                    line.setBackgroundColor(Color.parseColor("#D0D0D0"));

                    RelativeLayout.LayoutParams params = new RelativeLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 2);
                    params.addRule(RelativeLayout.BELOW, idCount - 1);
                    line.setLayoutParams(params);

                    insert.addView(line);
                }//if

                Map.Entry<String, Object> entry = globIter.next();
                Object value = entry.getValue();
                String key = entry.getKey();

                TextView label = new TextView(this);
                label.setSingleLine(true);
                label.setText(key);
                label.setTextSize(20f);
                label.setPadding(25, 10, 0, 0);
                label.setId(idCount++);

                RelativeLayout.LayoutParams labelParams = new RelativeLayout.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
                labelParams.setMargins(0, 20, 0, 20);
                labelParams.addRule(RelativeLayout.ALIGN_PARENT_LEFT);

                if (idCount > 0) {
                    labelParams.addRule(RelativeLayout.BELOW, idCount - 2);
                }//if

                label.setLayoutParams(labelParams);

                insert.addView(label);

                if (value instanceof Double) {
                    EditText field = new EditText(this);
                    field.setSingleLine(true);
                    field.setId(idCount++);
                    field.setInputType(InputType.TYPE_CLASS_NUMBER);
                    field.setKeyListener(DigitsKeyListener.getInstance("-0123456789."));
                    field.setText(value.toString());

                    RelativeLayout.LayoutParams params = new RelativeLayout.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
                    params.addRule(RelativeLayout.ALIGN_PARENT_RIGHT);
                    params.setMargins(0, 20, 10, 20);

                    if (idCount > 2) {
                        params.addRule(RelativeLayout.BELOW, idCount - 3);
                    }//if

                    field.setLayoutParams(params);
                    insert.addView(field);
                } else if (value instanceof Boolean) {
                    ToggleButton field = new ToggleButton(this);
                    field.setId(idCount++);

                    field.setTextOff("FALSE");
                    field.setTextOn("TRUE");

                    field.toggle();
                    field.toggle();

                    if ((((Boolean) value).booleanValue() && !field.isChecked())
                            || (!((Boolean) value).booleanValue() && field.isChecked())) {
                        field.toggle();
                    }//if

                    RelativeLayout.LayoutParams params = new RelativeLayout.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
                    params.addRule(RelativeLayout.ALIGN_PARENT_RIGHT);
                    params.setMargins(0, 20, 10, 20);

                    if (idCount > 2) {
                        params.addRule(RelativeLayout.BELOW, idCount - 3);
                    }//if

                    field.setLayoutParams(params);
                    insert.addView(field);
                } else if (value instanceof String) {
                    EditText field = new EditText(this);
                    field.setSingleLine(true);
                    field.setId(idCount++);
                    field.setText(value.toString());

                    RelativeLayout.LayoutParams params = new RelativeLayout.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
                    params.addRule(RelativeLayout.ALIGN_PARENT_RIGHT);
                    params.setMargins(0, 20, 10, 20);

                    if (idCount > 2) {
                        params.addRule(RelativeLayout.BELOW, idCount - 3);
                    }//if

                    field.setLayoutParams(params);
                    insert.addView(field);
                }//else

            }//for

            final int totalIds = idCount;

            Button submit = new Button(this);
            submit.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {

                    for (int i = 1; i < totalIds; i += 2) {
                        String key = (String) ((TextView) findViewById(i - 1)).getText();
                        View input = findViewById(i);

                        if (input instanceof EditText && ((EditText) input).getInputType() == (InputType.TYPE_CLASS_NUMBER)) {
                            double val = Double.parseDouble(((EditText) input).getText().toString());

                            globals.put(key, new Double(val));
                        } else if (input instanceof EditText) {
                            globals.put(key, ((EditText) input).getText().toString());
                        } else if (input instanceof ToggleButton) {
                            boolean val = ((ToggleButton) input).isChecked();
                            globals.put(key, new Boolean(val));

                        }//else

                    }//for

                    writeValuesToFile();

                    startActivity(new Intent(GlobalValuesActivity.this, FtcRobotControllerActivity.class));
                }//onClick
            });//onClickListener

            submit.setText("SUBMIT");

            RelativeLayout.LayoutParams params = new RelativeLayout.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
            params.addRule(RelativeLayout.CENTER_IN_PARENT);
            params.addRule(RelativeLayout.BELOW, idCount - 2);
            params.setMargins(0, 100, 0, 0);
            submit.setLayoutParams(params);

            insert.addView(submit);
        } else {

            TextView txt = new TextView(this);
            txt.setText("You haven't registered any global values!");

            RelativeLayout.LayoutParams params = new RelativeLayout.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
            params.addRule(RelativeLayout.CENTER_IN_PARENT);
            params.setMargins(10, 20, 10, 0);
            txt.setLayoutParams(params);

            insert.addView(txt);

        }//else

    }//onCreate

    public static void writeValuesToFile() {
        try {
            FileOutputStream globalsWrite = new FileOutputStream(new File(GLOBALS_FILE_PATH));

            for (Map.Entry<String, Object> entry : globals.entrySet()) {
                if (entry.getValue() instanceof Double) {
                    globalsWrite.write(("d,;" + entry.getKey() + ",;" + (Double) entry.getValue() + "\n").getBytes());
                } else if (entry.getValue() instanceof String) {
                    globalsWrite.write(("s,;" + entry.getKey() + ",;" + entry.getValue() + "\n").getBytes());
                } else {
                    globalsWrite.write(("b,;" + entry.getKey() + ",;" + entry.getValue() + "\n").getBytes());
                }//else
            }//for

            globalsWrite.close();
        } catch (Exception e) {
            e.printStackTrace();
        }//catch

    }//writeValuesToFile

    public static void add(String key, double val){
        globals.put(key, val);
    }//add
    public static void add(String key, boolean val){
        globals.put(key, val);
    }//add
    public static void add(String key, String val){
        globals.put(key, val);
    }//add

    //if key already exists, then we don't replace it
    public static void tentativelyAdd(String key, double val) {
        if (!globals.containsKey(key)) {
            globals.put(key, val);
        }//if
    }//recessivelyAdd

    public static void tentativelyAdd(String key, boolean val) {
        if (!globals.containsKey(key)) {
            globals.put(key, val);
        }//if
    }//recessivelyAdd

    public static void tentativelyAdd(String key, String val) {
        if (!globals.containsKey(key)) {
            globals.put(key, val);
        }//if
    }//recessivelyAdd


    public static void addDashboard(String key, String val) {
        tentativelyAdd(key, val);
        autoKeys.add(key);
    }//addDashboard


    public static void addDashboard(String key, double val) {
        tentativelyAdd(key, val);
        autoKeys.add(key);
    }//addDashboard


    public static void addDashboard(String key, boolean val) {
        tentativelyAdd(key, val);
        autoKeys.add(key);
    }//addDashboard


    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_global_values, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }
}
