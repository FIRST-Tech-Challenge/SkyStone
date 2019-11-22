package org.firstinspires.ftc.teamcode.auto.structurebuilder;

public class StructureConstructor {

    private Structure structure;

    public StructureConstructor(Structure structure){
        this.structure = structure;
    }


    public double getNextHeight(){
        if(structure.currentLayer().nextStone()){
            return structure.currentLayer().getCurrentStone().getZCoord() * 4 + 1;
        } else if(structure.nextLayer()){
            return structure.currentLayer().getCurrentStone().getZCoord() * 4 + 1;
        }
        return 0;
    }
}
