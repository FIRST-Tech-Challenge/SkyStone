package org.firstinspires.ftc.teamcode.Skystone.Modules;

import org.firstinspires.ftc.teamcode.Skystone.HardwareCollection;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class SpoolModule {

    public double spoolPower;
    public boolean autoLevel;

    public int level;
    public boolean alignHeight;

    public StringBuilder spoolData;

    public SpoolModule(){
        spoolPower = 0.0;
        autoLevel = false;
        level = 0;
        alignHeight = false;

        spoolData = new StringBuilder();
        spoolData.append("spoolPower level autoLevel alignHeight");
        spoolData.append("\n");
    }

    public synchronized void update(Robot robot, HardwareCollection hardwareCollection){

        if (robot.isDebug){
            spoolData.append(spoolPower);
            spoolData.append(" ");
            spoolData.append(level);
            spoolData.append(" ");
            spoolData.append(autoLevel);
            spoolData.append(" ");
            spoolData.append(alignHeight);
            spoolData.append("\n");
        }

        if (autoLevel){
            // implement spool logic here
        }

        // just set power to spoolPower
        hardwareCollection.outtakeSpool.setPower(spoolPower);
        hardwareCollection.outtakeSpool2.setPower(spoolPower);
    }

//    private void spoolLogic() {
//        spoolPosition = robot.getOuttakeSpool().getCurrentPosition();
//
//        if (gamepad2.dpad_down && !isG2DPDPushed) {
//            isMovingSpoolToPosition = true;
//            if (lastDropPosition == 0) {
//                spoolTargetPosition = 0;
//            } else {
//                if (isCapped) {
//                    spoolTargetPosition = spoolTargetPosition - 200;
//                } else {
//                    spoolTargetPosition = spoolTargetPosition - 150;
//                }
//            }
//
//            isG2DPDPushed = true;
//        } else if (!gamepad2.dpad_down) {
//            isG2DPDPushed = false;
//        }
//
//        if (gamepad2.right_bumper && !isTogglingRB) {
//            isTogglingRB = true;
//            double centerPosition = lastDropPosition;
//
//            if (lastDropPosition == 0) {
//                indexPosition = 0;
//            } else {
//                for (int i = 0; i < robot.spoolHeights.length; i++) {
//                    if (robot.spoolHeights[i] > centerPosition) {
//                        indexPosition = i + 1;
//                        break;
//                    }
//                }
//            }
//
//            spoolTargetPosition = robot.spoolHeights[indexPosition];
//
//            isMovingSpoolToPosition = true;
//        } else if (!gamepad2.right_bumper) {
//            isTogglingRB = false;
//        }
//
//        if (isMovingSpoolToPosition) {
//            if (spoolTargetPosition < 0) {
//                spoolTargetPosition = 0;
//            }
//            if (Math.abs(spoolPosition - spoolTargetPosition) < 50) {
//                spoolPower = .125;
//            } else if (spoolTargetPosition < spoolPosition) {
//                spoolPower = (spoolTargetPosition - spoolPosition) / 1500;
//            } else {
//                spoolPower = (spoolTargetPosition - spoolPosition) / 200;
//            }
//        }
//
//        //override mode
//        if (gamepad2.left_stick_y != 0) {
//            isMovingSpoolToPosition = false;
//            isTogglingRB = false;
//        }
//        if (!isMovingSpoolToPosition) {
//            spoolPower = -gamepad2.left_stick_y;
//        }
//        if (gamepad2.left_trigger != 0 && gamepad2.left_stick_y != 0) {
//            spoolPower = -gamepad2.left_stick_y / 10000;
//        } else if (gamepad2.left_trigger != 0) {
//            isTogglingRB = false;
//
//            spoolPower = .125;
//        }
//
//        if (spoolPosition >= 4200) {
//            if (spoolPower == 0) {
//                spoolPower = 0.125;
//            }
//            spoolPower = Math.min(spoolPower / 2, 0.15);
//        }
//
//        if (spoolPosition <= 0 && spoolPower < 0) {
//            spoolPower = 0;
//        }
//
//        if (gamepad2.left_bumper) {
//            spoolPower = -gamepad2.left_stick_y;
//            isOverridingSlideFloor = true;
//        } else if (isOverridingSlideFloor) {
//            isOverridingSlideFloor = false;
//            robot.getOuttakeSpool().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//
//
//        robot.getOuttakeSpool().setPower(spoolPower);
//        robot.getOuttakeSpool2().setPower(spoolPower);
//    }
}
