package org.firstinspires.ftc.teamcode.auto.structurebuilder.prefab;

import org.firstinspires.ftc.teamcode.auto.structurebuilder.Layer;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.Stone;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.Structure;

public class OneByOneByFive {

    public static Structure toStructure(){
        return new Structure.StructureBuilder().addLayer(new Layer.LayerBuilder().addStone(new Stone(0,0,2,1,0)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,2,1,1)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,2,1,2)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,2,1,3)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,2,1,4)).build()).build();
    }
}
