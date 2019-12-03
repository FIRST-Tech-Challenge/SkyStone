package org.firstinspires.ftc.teamcode.auto.structurebuilder;

public class StructureConstructor {

    private Structure structure;

    public StructureConstructor(Structure structure){
        this.structure = structure;
    }

    public double getCurrentHeight(){
        if(structure.currentLayer().getCurrentStone() != null){
            return structure.currentLayer().getCurrentStone().getZCoord() * 4 + 3.25;
        }
        return 0;
    }

    public double getNextHeight(){
        if(structure.currentLayer().nextStone()){
            return structure.currentLayer().getCurrentStone().getZCoord() * 4 + 3.25;
        } else if(structure.nextLayer()){
            return structure.currentLayer().getCurrentStone().getZCoord() * 4 + 3.25;
        }
        return 0;
    }
}
