package org.firstinspires.ftc.teamcode.auto.structurebuilder.prefab;

import org.firstinspires.ftc.teamcode.auto.structurebuilder.Layer;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.Stone;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.Structure;

public class TwoByTwoByFive {

    public static Structure toStructure(){
        return new Structure.StructureBuilder().addLayer(new Layer.LayerBuilder().addStone(new Stone(0,0,2,1,0)).addStone(new Stone(0,0,2,2,0)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,2,1,1)).addStone(new Stone(90,0,3,1,1)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(0,0,2,1,2)).addStone(new Stone(0,0,2,2,2)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,2,1,3)).addStone(new Stone(90,0,3,1,3)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(0,0,2,1,4)).addStone(new Stone(0,0,2,2,4)).build()).build();
    }
}
