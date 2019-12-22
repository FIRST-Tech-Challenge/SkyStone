package org.firstinspires.ftc.teamcode;


public class WaitForNavigationTask implements RobotControl {
    RobotNavigator navigator;
    RobotPosition pos1, pos2;
    boolean isDone = false;

    public WaitForNavigationTask(RobotNavigator navigator, RobotPosition pos1, RobotPosition pos2) {
        this.navigator = navigator;
        this.pos1 = pos1;
        this.pos2 = pos2;
    }

    @Override
    public void prepare() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void cleanUp() {

    }

    // isDone only when the navigator x,y,h all within the range of pos1 and pos2
    @Override
    public boolean isDone() {
        double ratioX = 0;
        double ratioY = 0;
        double ratioA = 0;
        if (pos1.getX() != pos2.getX()) {
            ratioX = (navigator.getWorldX() - pos1.getX())/(pos2.getX() - pos1.getX());
        }
        if (pos1.getY() != pos2.getY()) {
            ratioY = (navigator.getWorldY() - pos1.getY())/(pos2.getY() - pos1.getY());
        }
        if (pos1.getHeading() != pos2.getHeading()) {
            ratioA = (navigator.getHeading() - pos1.getHeading())/(pos2.getHeading() - pos1.getHeading());
        }
        return ratioX>0 && ratioX<1.0 && ratioY>0 && ratioY<1.0 && ratioA>0 && ratioA<1.0;
    }
}

