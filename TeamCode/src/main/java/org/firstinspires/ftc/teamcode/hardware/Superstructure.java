package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.auto.structurebuilder.StructureConstructor;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.prefab.OneByOneByFive;

public class Superstructure {

    private Elevator elevator;
    private Intake intake;
    private StructureConstructor structureConstructor;

    private SystemState currentState = SystemState.IDLE;

    private long stateTime = 0;

    public enum SystemState {
        IDLE, EMPTY, HOLDING, ABOVE_POSITION, PLACING, RELEASING, MANUAL
    }

    public Superstructure(Elevator elevator, Intake intake){
        this.elevator = elevator;
        this.intake = intake;
        structureConstructor = new StructureConstructor(new OneByOneByFive().toStructure());
        stateTime = System.currentTimeMillis();
    }

    public void update(){

        switch (currentState){
            case IDLE:
                elevator.stop();
                intake.stop();
                break;

            case EMPTY:
                if(elevator.getRelativeHeight() < -0.25 || elevator.getRelativeHeight() > 0.25){
                    elevator.setPosition(0);
                }
                break;

            case HOLDING:
                if(elevator.getCurrentState() != Elevator.SystemState.HOLD){
                    elevator.setHolding();
                }
                if(intake.getCurrentState() != Intake.State.GRABBING){
                    intake.setHold();
                }
                break;
        }
        elevator.update();
        intake.update();
    }

    public void handleStateTransition(SystemState state){
        switch (state){
            case EMPTY:
                currentState = SystemState.EMPTY;
                break;
            case HOLDING:
                currentState = SystemState.HOLDING;
                break;
            case ABOVE_POSITION:
                currentState = SystemState.ABOVE_POSITION;
                break;
            case IDLE:
                currentState = SystemState.IDLE;
                break;
            case PLACING:
                currentState = SystemState.PLACING;
                break;
            case MANUAL:
                currentState = SystemState.MANUAL;
                break;
        }
        resetTime();
    }

    public void grabStone(){
        handleStateTransition(SystemState.HOLDING);
    }

    public void goToPlacingPosition(){
        elevator.setPosition(structureConstructor.getCurrentHeight());
        handleStateTransition(SystemState.ABOVE_POSITION);
    }

    public void placeStone(){
        elevator.lowerToPlace();
        handleStateTransition(SystemState.PLACING);
    }

    public void releaseStone(){
        intake.open();
        handleStateTransition(SystemState.RELEASING);
    }

    public void setManual(){
        handleStateTransition(SystemState.MANUAL);
        elevator.setDriverControlled();
    }

    public void doUpAction(){
        switch (currentState){
            case EMPTY:
                grabStone();
                break;

            case MANUAL:
                grabStone();
                break;

            case HOLDING:
                goToPlacingPosition();
                break;

            default:
                handleStateTransition(currentState);
        }
    }

    public void doDownAction(){
        switch (currentState){
            case ABOVE_POSITION:
                placeStone();
                break;

            case PLACING:
                releaseStone();
                break;

            case RELEASING:
                handleStateTransition(SystemState.EMPTY);
                structureConstructor.getNextHeight();
                break;

                default:
                    handleStateTransition(currentState);
        }
    }

    private long getElapsedTime(){
        return System.currentTimeMillis() - stateTime;
    }

    private void resetTime(){
        stateTime = System.currentTimeMillis();
    }

    public SystemState getCurrentState(){
        return currentState;
    }

}
