
package org.firstinspires.ftc.teamcode.CustomCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class MainPipeline extends OpenCvPipeline {

    private Mat mat0;
    private Mat mat1;
    private Mat mat2;

    private Mat mask0;
    private Mat mask1;
    private Mat mask2;

    private boolean madeMats = false;

    private Scalar BLACK = new Scalar(0,0,0);
    private Scalar WHITE = new Scalar(255,255,255);
    private Scalar RED = new Scalar(255, 0, 0);

    //X is actually Y
    private double cx0 = 135;
    private double cy0 = 200;

    private double cx1 = 135;
    private double cy1 = 140;

    private double cx2 = 135;
    private double cy2 = 80;

    private int r = 6;
    private int strokeWidth = 3;

    public SkystoneLocation location = SkystoneLocation.right;

    @Override
    public Mat processFrame(Mat frame)
    {
        int h = frame.height();
        int w = frame.width();

        int type = frame.type();
        if (!madeMats) {
            mask0 = new Mat(h, w, type);
            mask1 = new Mat(h, w, type);
            mask2 = new Mat(h, w, type);
            mat0 = new Mat();
            mat1 = new Mat();
            mat2 = new Mat();
            madeMats = true;
        }

        mask0.setTo(BLACK);
        mask1.setTo(BLACK);
        mask2.setTo(BLACK);

        Imgproc.circle(mask0, new Point(cx0, cy0), r, WHITE, Core.FILLED);
        Imgproc.circle(mask1, new Point(cx1, cy1), r, WHITE, Core.FILLED);
        Imgproc.circle(mask2, new Point(cx2, cy2), r, WHITE, Core.FILLED);

        Core.bitwise_and(mask0, frame, mat0);
        Core.bitwise_and(mask1, frame, mat1);
        Core.bitwise_and(mask2, frame, mat2);

        double val0 = Core.sumElems(mat0).val[0] + Core.sumElems(mat0).val[1] + Core.sumElems(mat0).val[2];
        double val1 = Core.sumElems(mat1).val[0] + Core.sumElems(mat1).val[1] + Core.sumElems(mat1).val[2];
        double val2 = Core.sumElems(mat2).val[0] + Core.sumElems(mat2).val[1] + Core.sumElems(mat2).val[2];

        if (val0 < val1 && val0 < val2) {
            location = SkystoneLocation.right;
        } else if (val1 < val0 && val1 < val2) {
            location = SkystoneLocation.middle;
        } else {
            location = SkystoneLocation.left;
        }

        Scalar s0 = WHITE;
        Scalar s1 = WHITE;
        Scalar s2 = WHITE;

        if (location == SkystoneLocation.right) {
            s0 = RED;
        } else if (location == SkystoneLocation.left) {
            s2 = RED;
        } else {
            s1 = RED;
        }

        Imgproc.line(frame, new Point(0, 275), new Point(300, 275), new Scalar(0, 255, 0));
        Imgproc.circle(frame, new Point(cx0, cy0), r, s0, Core.FILLED);
        Imgproc.circle(frame, new Point(cx1, cy1), r, s1, Core.FILLED);
        Imgproc.circle(frame, new Point(cx2, cy2), r, s2, Core.FILLED);

        return frame;
    }

    public int getSkystonePosition() {
        if(location == SkystoneLocation.left) {
            return 0;
        }else if(location == SkystoneLocation.middle) {
            return 1;
        }else if(location == SkystoneLocation.right) {
            return 2;
        }else{
            return 404;
        }
    }
}
