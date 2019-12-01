package org.firstinspires.ftc.teamcode.Autonomous.Vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Tensorflow.TFODCalc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Detect {

    public Detect() {
        TFODCalc.init();
        TFODCalc.setHardwareProperties(43.30, 3.67f);
    }

    public int[] getSkystonePositionsBlue(List<Recognition> updatedRecognitions, double imageWidthPx) {    //Stones left -> right
        if (updatedRecognitions != null) {
            int index = 0;
            double[] left = new double[updatedRecognitions.size()];
            ArrayList<String> skystoneIndex = new ArrayList<>();

            for (Recognition recognition : updatedRecognitions) {
                left[index] = recognition.getLeft();
                if (recognition.getLabel().equalsIgnoreCase("skystone"))
                    skystoneIndex.add("skystone");
                else
                    skystoneIndex.add("stone");
                index += 1;
            }

            switch (updatedRecognitions.size()) {
                case 1:
                    if (skystoneIndex.get(0).equalsIgnoreCase("skystone")) {
                        double horizontalMid = updatedRecognitions.get(0).getLeft() + updatedRecognitions.get(0).getWidth() / 2;
                        double dividedImg = imageWidthPx / 4;

                        if (horizontalMid <= dividedImg)
                            return new int[]{1, 4};
                        else if (horizontalMid > dividedImg && horizontalMid <= dividedImg * 2)
                            return new int[]{2, 5};
                        else
                            return new int[]{3, 6};
                    } else
                        return new int[]{1, 4};
                case 2:
                    if (!skystoneIndex.contains("skystone")) {
                        return new int[]{3, 6};
                    } else {
                        if (left[skystoneIndex.indexOf("skystone")] > left[skystoneIndex.indexOf("stone")])
                            return new int[]{2, 5};
                        else if (left[skystoneIndex.indexOf("skystone")] <= left[skystoneIndex.indexOf("stone")])
                            return new int[]{1, 4};
                    }
                    break;
            }

            if (skystoneIndex.contains("skystone")) {
                double minPos = 9999;
                if (updatedRecognitions.size() >= 3) {
                    for (int x = 0; x < skystoneIndex.size(); x++) {
                        if (skystoneIndex.get(x).equalsIgnoreCase("skystone")) {
                            if (minPos > updatedRecognitions.get(x).getLeft())
                                minPos = updatedRecognitions.get(x).getLeft();
                        }
                    }

                    int idx = 0;
                    for (Recognition r : updatedRecognitions) {
                        if (r.getLeft() < minPos)
                            idx += 1;
                    }
                    return new int[]{idx + 1, idx + 4};
                }
            }
        }
        return new int[]{-1, -1};
    }

    public int[] getSkystonePositionsRed(List<Recognition> updatedRecognitions, double imageWidthPx) {     //Stones right -> left
        if (updatedRecognitions != null) {
            int index = 0;
            double[] right = new double[updatedRecognitions.size() + 1];
            ArrayList<String> skystoneIndex = new ArrayList<>();

            for (Recognition recognition : updatedRecognitions) {
                right[index] = recognition.getRight();
                if (recognition.getLabel().equalsIgnoreCase("skystone"))
                    skystoneIndex.add("skystone");
                else
                    skystoneIndex.add("stone");
                index += 1;
            }

            switch (updatedRecognitions.size()) {
                case 1:
                    if (skystoneIndex.get(0).equalsIgnoreCase("skystone")) {
                        double horizontalMid = updatedRecognitions.get(0).getLeft() + updatedRecognitions.get(0).getWidth() / 2;
                        double dividedImg = imageWidthPx / 4;

                        if (horizontalMid >= dividedImg * 3)
                            return new int[]{1, 4};
                        else if (horizontalMid < dividedImg * 3 && horizontalMid >= dividedImg * 2)
                            return new int[]{2, 5};
                        else
                            return new int[]{3, 6};
                    } else
                        return new int[]{1, 4};
                case 2:
                    if (!skystoneIndex.contains("skystone")) {
                        return new int[]{3, 6};
                    } else {
                        if (right[skystoneIndex.indexOf("skystone")] > right[skystoneIndex.indexOf("stone")])
                            return new int[]{1, 4};
                        else if (right[skystoneIndex.indexOf("skystone")] <= right[skystoneIndex.indexOf("stone")])
                            return new int[]{2, 5};
                    }
                    break;
            }

            if (skystoneIndex.contains("skystone")) {
                double maxPos = -9999;
                if (updatedRecognitions.size() >= 3) {
                    for (int x = 0; x < skystoneIndex.size(); x++) {
                        if (skystoneIndex.get(x).equalsIgnoreCase("skystone")) {
                            if (maxPos < right[x])
                                maxPos = right[x];
                        }
                    }

                    int idx = 0;
                    for (double r : right) {
                        if (r > maxPos)
                            idx += 1;
                    }
                    return new int[]{idx + 1, idx + 4};
                }
            }
        }
        return new int[]{-1, -1};
    }
}
