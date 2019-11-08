package org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;

public class XYPlaneCalculations {
    public static double[] rotatePointAroundFixedPoint_Deg(double[] point, double[] fixedPoint, double counterClockwiseAng) {
        double relativeY = point[1] - fixedPoint[1], relativeX = point[0] - fixedPoint[0];
        double deltaAng = Math.toRadians(counterClockwiseAng);
        double sinDeltaAng = Math.sin(deltaAng);
        double cosDeltaAng = Math.cos(deltaAng);
        double newX = relativeX * cosDeltaAng - relativeY * sinDeltaAng;
        double newY = relativeX * sinDeltaAng + relativeY * cosDeltaAng;
        double[] result = {newX + fixedPoint[0], newY + fixedPoint[1]};
        return result;
    }
    public static double[] rotatePointAroundFixedPoint_Rad(double[] point, double[] fixedPoint, double counterClockwiseAng){
        double relativeY = point[1] - fixedPoint[1], relativeX = point[0] - fixedPoint[0];
        double deltaAng = counterClockwiseAng;
        double sinDeltaAng = Math.sin(deltaAng);
        double cosDeltaAng = Math.cos(deltaAng);
        double newX = relativeX * cosDeltaAng - relativeY * sinDeltaAng;
        double newY = relativeX * sinDeltaAng + relativeY * cosDeltaAng;
        double[] result = {newX + fixedPoint[0], newY + fixedPoint[1]};
        return result;
    }
    public static Robot3DPositionIndicator getFTCRobotPosition(Robot3DPositionIndicator DarbotsRobotPosition){
        return new Robot3DPositionIndicator(
                DarbotsRobotPosition.getZ(),
                -DarbotsRobotPosition.getX(),
                DarbotsRobotPosition.getY(),
                DarbotsRobotPosition.getRotationZ(),
                -DarbotsRobotPosition.getRotationX(),
                DarbotsRobotPosition.getRotationY() - 90
        );
    }

    public static Robot3DPositionIndicator getDarbotsRobotPosition(Robot3DPositionIndicator FTCRobotPosition){
        return new Robot3DPositionIndicator(
                -FTCRobotPosition.getY(),
                FTCRobotPosition.getZ(),
                FTCRobotPosition.getX(),
                -FTCRobotPosition.getRotationY(),
                FTCRobotPosition.getRotationZ() + 90,
                FTCRobotPosition.getRotationX());
    }

    public static Robot2DPositionIndicator getFTCRobotPosition(Robot2DPositionIndicator DarbotsRobotPosition){
        return new Robot2DPositionIndicator(
                DarbotsRobotPosition.getZ(),
                -DarbotsRobotPosition.getX(),
                DarbotsRobotPosition.getRotationY() - 90
        );
    }

    public static Robot2DPositionIndicator getDarbotsRobotPosition(Robot2DPositionIndicator FTCRobotPosition){
        return new Robot2DPositionIndicator(
                -FTCRobotPosition.getZ(),
                FTCRobotPosition.getX(),
                FTCRobotPosition.getRotationY()+90
        );
    }

    public static Robot2DPositionIndicator getRelativePosition(Robot2DPositionIndicator PerspectiveOrigin, Robot2DPositionIndicator Target){
        //First step - move the Perspective Origin to the Origin of the Axis.
        double[] targetPoint = {Target.getX() - PerspectiveOrigin.getX(),Target.getZ() - PerspectiveOrigin.getZ()};
        //Second step - rotate the targetPoint so that the coordinate system (X and Z scalars) of the Perspective Origin overlaps with the Field Coordinate.
        //We are basically rotating field coordinate here.
        double[] origin = {0,0};
        double[] rotatedTargetPoint = rotatePointAroundFixedPoint_Deg(targetPoint,origin,-PerspectiveOrigin.getRotationY());
        //Third step - calculate relative delta Rotation Y;
        double deltaRotY = normalizeDeg(Target.getRotationY() - PerspectiveOrigin.getRotationY());

        return new Robot2DPositionIndicator(rotatedTargetPoint[0],rotatedTargetPoint[1],deltaRotY);
    }

    public static Robot2DPositionIndicator getAbsolutePosition(Robot2DPositionIndicator PerspectiveOrigin, Robot2DPositionIndicator RelativePosition){
        //First Step - calculate absolute Rotation Y.
        double absRotY = normalizeDeg(RelativePosition.getRotationY() + PerspectiveOrigin.getRotationY());
        //Second Step - rotate the coordinates back.
        double[] origin = {0,0};
        double[] relativeTargetPoint = {RelativePosition.getX(),RelativePosition.getZ()};
        double[] rotatedTargetPoint = rotatePointAroundFixedPoint_Deg(relativeTargetPoint,origin,PerspectiveOrigin.getRotationY());
        //Third Step - move the PerspectiveOrigin back to the Absolute Point on the Field.
        double[] movedTargetPoint = {rotatedTargetPoint[0] + PerspectiveOrigin.getX(),rotatedTargetPoint[1] + PerspectiveOrigin.getZ()};

        return new Robot2DPositionIndicator(movedTargetPoint[0],movedTargetPoint[1],absRotY);
    }

    public static double chooseAngleFromRange(double[] angleList, double angleSmallestRange, double angleBiggestRange) {
        for(int i=0;i<angleList.length;i++) {
            if(angleList[i] >= angleSmallestRange && angleList[i] <= angleBiggestRange) {
                return angleList[i];
            }
        }
        return angleList[0];
    }

    public static double normalizeRad(double Rad) {
        double tempRad = Rad % (Math.PI * 2);
        if(tempRad >= Math.PI){
            tempRad -= (Math.PI) * 2;
        }
        return tempRad;
    }

    public static float normalizeRad(float Rad){
        float tempRad = (float) (Rad % (Math.PI * 2));
        if(tempRad >= Math.PI){
            tempRad -= (Math.PI) * 2;
        }
        return tempRad;
    }

    public static double normalizeDeg(double Deg) {
        double tempDeg = Deg % 360;
        if(tempDeg >= 180){
            tempDeg -= 360;
        }
        return tempDeg;
    }

    public static float normalizeDeg(float Deg) {
        float tempDeg = Deg % 360;
        if(tempDeg >= 180){
            tempDeg -= 360;
        }
        return tempDeg;
    }
}