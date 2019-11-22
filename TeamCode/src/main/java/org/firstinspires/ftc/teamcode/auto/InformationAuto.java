package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.structurebuilder.Structure;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.prefab.TwoByTwoByFive;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

@Autonomous(name="Information Auto")
public class InformationAuto extends LinearOpMode {

    private static boolean redAlliance = true;

    private static DataChoice currentDataChoice = DataChoice.ALLIANCE;
    private static Structure structure = new TwoByTwoByFive().toStructure();

    private boolean gamepadPressed = false;
    private boolean bumperPressed = false;

    private static final String FILE_NAME = "AutoData.txt";

    private interface DataPoint {
        void setNextDataChoice();
    }

    private enum DataChoice implements DataPoint {
        ALLIANCE {
            public void setNextDataChoice() {
                currentDataChoice = DataChoice.STRUCTURETYPE;
            }
        }, STRUCTURETYPE{
            public void setNextDataChoice() {
                currentDataChoice = DataChoice.ALLIANCE;
            }
        }
    }

    public void runOpMode() {
        try {
            FileInputStream fileIn = new FileInputStream(FILE_NAME);
            InputStreamReader InputRead = new InputStreamReader(fileIn);

            char[] inputBuffer = new char[100];
            String s = "";
            int charRead;

            while ((charRead = InputRead.read(inputBuffer)) > 0) {

                String readstring = String.copyValueOf(inputBuffer, 0, charRead);
                s += readstring;
            }
            InputRead.close();
            if (s.contains("Red")) {
                redAlliance = true;
            } else if (s.contains("Blue")) {
                redAlliance = false;
            }
        } catch (Exception f) {
            f.printStackTrace();
        }

        while (!isStopRequested()) {
            if (redAlliance) {
                telemetry.addData("Current Alliance: ", "Red Alliance");
            } else {
                telemetry.addData("Current Alliance: ", "Blue Alliance");
            }
            if (gamepad1.a == true && !gamepadPressed) {
                currentDataChoice.setNextDataChoice();
                gamepadPressed = true;
            } else if (gamepad1.a == false) {
                gamepadPressed = false;
            }
            switch (currentDataChoice) {
                case ALLIANCE:
                    if (gamepad1.right_bumper && bumperPressed == false) {
                        redAlliance = !redAlliance;
                        bumperPressed = true;
                    } else if (!gamepad1.right_bumper) {
                        bumperPressed = false;
                    }
                    break;
                case STRUCTURETYPE: {
                    if (gamepad1.right_bumper && bumperPressed == false) {
                        redAlliance = !redAlliance;
                        bumperPressed = true;
                    } else if (!gamepad1.right_bumper) {
                        bumperPressed = false;
                    }
                }
                break;
            }
                try {
                    FileOutputStream fileos = new FileOutputStream(FILE_NAME);
                    OutputStreamWriter outputWriter = new OutputStreamWriter(fileos);
                    String s = "";
                    if (redAlliance) {
                        s = "Red";
                    } else {
                        s = "Blue";
                    }
                    outputWriter.write(s);
                    outputWriter.close();

                } catch (Exception e) {
                    e.printStackTrace();
                }
                telemetry.update();
            }
        }

        public static boolean ifRedAlliance () {
            try {
                FileInputStream fileIn = new FileInputStream(FILE_NAME);
                InputStreamReader InputRead = new InputStreamReader(fileIn);

                char[] inputBuffer = new char[100];
                String s = "";
                int charRead;

                while ((charRead = InputRead.read(inputBuffer)) > 0) {

                    String readstring = String.copyValueOf(inputBuffer, 0, charRead);
                    s += readstring;
                }
                InputRead.close();
                if (s.contains("Red")) {
                    redAlliance = true;
                } else if (s.contains("Blue")) {
                    redAlliance = false;
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
            return redAlliance;
        }
    }
