package org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation;

public class XYPlaneCalculations {
    public static double[] rotatePointAroundFixedPoint_Deg(double[] point, double[] fixedPoint, double counterClockwiseAng) {
        double relativeY = point[1] - fixedPoint[1], relativeX = point[0] - fixedPoint[0];
        double deltaAng = Math.toRadians(counterClockwiseAng);
        double newX = relativeX * Math.cos(deltaAng) - relativeY * Math.sin(deltaAng);
        double newY = relativeX * Math.sin(deltaAng) + relativeY * Math.cos(deltaAng);
        double[] result = {newX + fixedPoint[0], newY + fixedPoint[1]};
        return result;
    }
    public static double[] rotatePointAroundFixedPoint_Rad(double[] point, double[] fixedPoint, double counterClockwiseAng){
        double relativeY = point[1] - fixedPoint[1], relativeX = point[0] - fixedPoint[0];
        double deltaAng = counterClockwiseAng;
        double newX = relativeX * Math.cos(deltaAng) - relativeY * Math.sin(deltaAng);
        double newY = relativeX * Math.sin(deltaAng) + relativeY * Math.cos(deltaAng);
        double[] result = {newX + fixedPoint[0], newY + fixedPoint[1]};
        return result;
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
    public static double normalizeDeg(double Deg) {
        double tempDeg = Deg % 360;
        if(tempDeg >= 180){
            tempDeg -= 360;
        }
        return tempDeg;
    }
}