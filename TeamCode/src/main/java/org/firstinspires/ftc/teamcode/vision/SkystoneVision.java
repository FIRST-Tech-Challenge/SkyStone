package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SkystoneVision extends OpenCvPipeline {

    private Mat yCbCrChan2Mat = new Mat();
    private Mat stoneThreshold = new Mat();
    private Mat stoneErode = new Mat();
    private Mat stoneDilate = new Mat();
    private Mat stoneContours = new Mat();
    private Mat skystoneThreshold = new Mat();
    private Mat skystoneErode = new Mat();
    private Mat skystoneDilate = new Mat();
    private Mat skystoneContours = new Mat();
    private List<MatOfPoint> skystoneContoursList = new ArrayList<>();
    private List<MatOfPoint> stoneCountoursList = new ArrayList<>();
    private ArrayList<Rect> stoneRectangles = new ArrayList<>();
    private ArrayList<Rect> skystoneRectangles = new ArrayList<>();
    private SkystonePosition.Positions skystonePosition = SkystonePosition.Positions.UNKNOWN;

    enum Stage
    {
        YCbCr_CHAN2,
        STONE_THRESHOLD,
        STONE_ERODE,
        STONE_DILATE,
        CONTOURS_OVERLAYED_ON_FRAME,
        RAW_IMAGE,
        SKYSTONE_THRESHOLD,
        SKYSTONE_ERODE,
        SKYSTONE_DILATE,
        SKYSTONE_CONTOURS,
    }

    private Stage stageToRenderToViewport = Stage.SKYSTONE_CONTOURS;

    @Override
    public Mat processFrame(Mat input){
        stoneCountoursList.clear();

        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
        Imgproc.threshold(yCbCrChan2Mat, stoneThreshold, 102, 255, Imgproc.THRESH_BINARY_INV);

        Mat cvErodeKernel = new Mat();
        Point cvErodeAnchor = new Point(-1, -1);
        double cvErodeIterations = 20.0;
        int cvErodeBordertype = Core.BORDER_CONSTANT;
        Scalar cvErodeBordervalue = new Scalar(-1);
        cvErode(stoneThreshold, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, stoneErode);

        Mat cvDilateSrc = stoneErode;
        Mat cvDilateKernel = new Mat();
        Point cvDilateAnchor = new Point(-1, -1);
        double cvDilateIterations = 15.0;
        int cvDilateBordertype = Core.BORDER_CONSTANT;
        Scalar cvDilateBordervalue = new Scalar(-1);
        cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, stoneDilate);

        Imgproc.findContours(stoneDilate, stoneCountoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        input.copyTo(stoneContours);
        Imgproc.drawContours(stoneContours, stoneCountoursList, -1, new Scalar(0, 0, 255), 3, 8);

        Mat hsvThresholdInput = input;
        double[] hsvThresholdHue = {0.0, 180.0};
        double[] hsvThresholdSaturation = {0.0, 255.0};
        double[] hsvThresholdValue = {0.0, 72.23549488054607};

        hsvThreshold(hsvThresholdInput,hsvThresholdHue,hsvThresholdSaturation,hsvThresholdValue,skystoneThreshold);

        double cvSkystoneErodeIterations = 6.0;
        cvErode(skystoneThreshold, cvErodeKernel, cvErodeAnchor, cvSkystoneErodeIterations, cvErodeBordertype, cvErodeBordervalue, skystoneErode);

        double cvSkystoneDilateIterations = 16.0;
        cvDilate(skystoneErode, cvDilateKernel, cvDilateAnchor, cvSkystoneDilateIterations, cvDilateBordertype, cvDilateBordervalue, skystoneDilate);
        findContours(skystoneDilate, false, skystoneContoursList);
        stoneContours.copyTo(skystoneContours);
        Imgproc.drawContours(skystoneContours,skystoneContoursList,-1,new Scalar(255,255,0),3,8);

        skystoneRectangles = getBoundingRect(skystoneContoursList);
        stoneRectangles = getBoundingRect(stoneCountoursList);

        for(Rect r : skystoneRectangles){
            Imgproc.rectangle(skystoneContours,r,new Scalar(255,235,132),3);
        }

        for(Rect r : stoneRectangles){
            Imgproc.rectangle(skystoneContours,r,new Scalar(134,235,32),3);
        }

        switch (stageToRenderToViewport)
        {
            case YCbCr_CHAN2:
            {
                return yCbCrChan2Mat;
            }

            case STONE_THRESHOLD:
            {
                return stoneThreshold;
            }

            case STONE_DILATE:{
                return stoneDilate;
            }

            case STONE_ERODE:{
                return stoneErode;
            }

            case CONTOURS_OVERLAYED_ON_FRAME:
            {
                return stoneContours;
            }

            case RAW_IMAGE:
                {
                return input;
            }

            case SKYSTONE_THRESHOLD:{
                return skystoneThreshold;
            }

            case SKYSTONE_ERODE:{
                return skystoneErode;
            }

            case SKYSTONE_DILATE:{
                return skystoneDilate;
            }

            case SKYSTONE_CONTOURS:{
                return skystoneContours;
            }

            default:
            {
                return input;
            }
        }
    }

    private static void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
                                 int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null){
            borderValue = new Scalar(-1);
        }
        Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    private static void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
                                int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null) {
            borderValue = new Scalar(-1);
        }
        Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    private void findContours(Mat input, boolean externalOnly,
                              List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

    private ArrayList<Rect> getBoundingRect(List<MatOfPoint> listContours){
        MatOfPoint2f approxCurve = new MatOfPoint2f();
        ArrayList<Rect> output = new ArrayList<>();

        for (int i=0; i<listContours.size(); i++) {
            //Convert contours(i) from MatOfPoint to MatOfPoint2f
            MatOfPoint2f contour2f = new MatOfPoint2f( listContours.get(i).toArray() );
            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true)*0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint( approxCurve.toArray() );

            // Get bounding rect of contour
            output.add(Imgproc.boundingRect(points));

        }
        return output;
    }

    public SkystonePosition.Positions getSkystonePosition(){
        for(Rect s : stoneRectangles){
            if(s == null){
                stoneRectangles.remove(s);
            }
        }
        if(stoneRectangles.size() >= 2){
            Rect largestRectangle1 = new Rect();
            Rect largestRectangle2 = new Rect();

            for(Rect r : stoneRectangles){
                if(r != null && r.area() > largestRectangle1.area()){
                    largestRectangle2 = largestRectangle1;
                    largestRectangle1 = r;
                } else if(r != null && r.area() > largestRectangle2.area()){
                    largestRectangle2 = r;
                }
            }

            if(skystoneRectangles.size() > 1){
                Rect largestSkystoneRect = new Rect();

                for(Rect r : skystoneRectangles){
                    if(r != null && r.area() > largestSkystoneRect.area()){
                        largestSkystoneRect = r;
                    }
                }

                double skystoneMidX = (2 * largestSkystoneRect.x + largestSkystoneRect.width)/2.0;
                double stone1MidX = (2 * largestRectangle1.x + largestSkystoneRect.width)/2.0;
                double stone2MidX = (2 * largestRectangle2.x+ largestRectangle2.width)/2.0;

                if(skystoneMidX > stone1MidX && skystoneMidX > stone2MidX){
                    skystonePosition = SkystonePosition.Positions.RIGHT;
                } else if(skystoneMidX < stone1MidX && skystoneMidX < stone2MidX){
                    skystonePosition = SkystonePosition.Positions.LEFT;
                } else if((skystoneMidX > stone1MidX && skystoneMidX < stone2MidX) || (skystoneMidX < stone1MidX && skystoneMidX > stone2MidX)){
                    skystonePosition = SkystonePosition.Positions.MIDDLE;
                }
            }
        } else if(stoneRectangles.size() == 1){
            Rect largestRectangle1 = new Rect();

            for(Rect r : stoneRectangles){
                if(r != null && r.area() > largestRectangle1.area()){
                    largestRectangle1 = r;
                }
            }

            if(skystoneRectangles.size() > 1){
                Rect largestSkystoneRect = new Rect();

                for(Rect r : skystoneRectangles){
                    if(r != null && r.area() > largestSkystoneRect.area()){
                        largestSkystoneRect = r;
                    }
                }

                double skystoneMidX = (2 * largestSkystoneRect.x + largestSkystoneRect.width)/2.0;
                double stone1MidX = (2 * largestRectangle1.x + largestSkystoneRect.width)/2.0;

                if(skystoneMidX > stone1MidX){
                    skystonePosition = SkystonePosition.Positions.RIGHT;
                } else if(skystoneMidX < stone1MidX){
                    skystonePosition = SkystonePosition.Positions.LEFT;
                }
            }
        }
        return skystonePosition;
    }
}
