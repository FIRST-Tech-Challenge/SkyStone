package org.firstinspires.ftc.teamcode;

public class Navigation {
    final double ENCODER_WHEEL_DIAMETER = 4.8;
    final int ENCODER_COUNTS_PER_ROTATION = 1440;
    final double ROBOT_WHEELBASE_WIDTH = 36.83; //unit in cm
    Position position = new Position();

    public double calculateAngle(double left_encoder, double right_encoder){
        double encoder_difference = convertEncoderCountsToCentimeters( right_encoder - left_encoder);
        double angle =  (180 * (encoder_difference))/(Math.PI * ROBOT_WHEELBASE_WIDTH);
        return angleWrap(angle);
    }

    //William Gutrich 10/13/19, Set all positions
    public void setAllPosition(double leftEncoder, double rightEncoder, double horEncoder, double angle) {
        double x, y = 0;

        if (leftEncoder != rightEncoder) {
            double x1 = leftEncoder * Math.sin(angle);
            double y1 = leftEncoder * Math.cos(angle);
            double x2 = horEncoder * Math.cos(angle);
            double y2 = horEncoder * Math.sin(angle);

            x = x1 + x2;
            y = y1 - y2;
        } else {
            x = convertEncoderCountsToCentimeters(horEncoder);
            if(leftEncoder-rightEncoder == 0 &&leftEncoder!= 0)
                y = convertEncoderCountsToCentimeters  (leftEncoder);

        }


        double hypot = Math.hypot(x, y);

        int sign = (leftEncoder-rightEncoder)<0 && rightEncoder<0 ? -1 : 1;
        double relativeX = sign*hypot*Math.sin(Math.toRadians(position.getRelativeAngle()+angle));
        double relativeY = sign*hypot* Math.cos(Math.toRadians(position.getRelativeAngle()+angle));

        position.setRelativeY(relativeY + position.getRelativeY());
        position.setRelativeX(relativeX + position.getRelativeX());

        double worldY = relativeY * Math.sin(Math.toRadians(angle)) +
                        relativeX *Math.cos(Math.toRadians(angle));
        double worldX = relativeY * Math.cos(Math.toRadians(angle)) +
                        relativeX * Math.sin(Math.toRadians(angle));
        position.setWorldY(worldY - position.getInitY());
        position.setWorldX(worldX - position.getInitX());
        position.setWorldAngle(Math.atan2(worldY,worldX));
    }

    //Alejandra, Athena, Marianna update the world position 10/11/19
    public void  calculateWorldX(){
        double pX = position.getWorldPreviousX();
        double rX = position.getRelativeX();
        position.setWorldX(pX + rX);
    }

    public void calculateWorldY () {
        double pY = position.getWorldPreviousY();
        double rY = position.getRelativeY();
        position.setWorldY(pY + rY);
    }

    public void calculateWorldAngle () {
        double pA = position.getWorldPreviousAngle();
        double rA = position.getRelativeAngle();
        position.setWorldAngle(pA + rA);
    }
    // October 6th: Lucas and Marianna
    public double convertEncoderCountsToCentimeters(double encoderCounts) {
        return encoderCounts*((ENCODER_WHEEL_DIAMETER*Math.PI)/ENCODER_COUNTS_PER_ROTATION);
    }

    public void findDisplacement() {

    }

    //10/14 Gabriel, limit angle range -180 to 180
    public double angleWrap(double angle) {
        if (angle > 180)
            angle = angle - 360;
        else if (angle < -180)
            angle = angle + 360;
        return angle;
    }
}
