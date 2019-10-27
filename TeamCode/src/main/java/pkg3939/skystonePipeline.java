package pkg3939;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class skystonePipeline extends OpenCvPipeline
{
    float offsetX = 0;
    float offsetY = 0;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static int[] vals = {valMid, valLeft, valRight};

    private float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};

    Mat yCbCrChan2Mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat all = new Mat();

    public skystonePipeline(float offsetX, float offsetY) {
        this.offsetX = offsetX;
        this.offsetY = offsetY;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        //color diff cb.
        //lower cb = more blue = skystone = white
        //higher cb = less blue = yellow stone = grey
        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

        //b&w
        Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

        yCbCrChan2Mat.copyTo(all);//copies mat object

        updateVals(input);

        //create three points
        Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
        Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
        Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

        //draw 2 circles
        drawCircles(pointMid, pointLeft, pointRight);

        //draw 3 rectangles
        drawRects();


        return all;
    }

    public void updateVals() {
        vals[0] = valMid;
        vals[1] = valLeft;
        vals[2] = valRight;
    }

    public int[] getVals() {
        return vals;
    }

    public void updateVals(Mat input) {
        //get values from frame
        double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
        valMid = (int)pixMid[0];

        double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
        valLeft = (int)pixLeft[0];

        double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
        valRight = (int)pixRight[0];
    }

    public void drawCircles(Point pointMid, Point pointLeft, Point pointRight) {

        Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
        Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
        Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle
    }

    //draw 3 rectangles
    public void drawRects() {
        Imgproc.rectangle(//1-3
                all,
                new Point(
                        all.cols()*(leftPos[0]-rectWidth/2),
                        all.rows()*(leftPos[1]-rectHeight/2)),
                new Point(
                        all.cols()*(leftPos[0]+rectWidth/2),
                        all.rows()*(leftPos[1]+rectHeight/2)),
                new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(//3-5
                all,
                new Point(
                        all.cols()*(midPos[0]-rectWidth/2),
                        all.rows()*(midPos[1]-rectHeight/2)),
                new Point(
                        all.cols()*(midPos[0]+rectWidth/2),
                        all.rows()*(midPos[1]+rectHeight/2)),
                new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(//5-7
                all,
                new Point(
                        all.cols()*(rightPos[0]-rectWidth/2),
                        all.rows()*(rightPos[1]-rectHeight/2)),
                new Point(
                        all.cols()*(rightPos[0]+rectWidth/2),
                        all.rows()*(rightPos[1]+rectHeight/2)),
                new Scalar(0, 255, 0), 3);
    }

}