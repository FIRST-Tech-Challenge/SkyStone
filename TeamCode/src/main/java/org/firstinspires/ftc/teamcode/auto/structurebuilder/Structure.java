package org.firstinspires.ftc.teamcode.auto.structurebuilder;

import java.util.ArrayList;

public class Structure {

    private ArrayList<Layer> layers;
    private int currentLayer;


    public static class StructureBuilder {

        private ArrayList<Layer> constructionLayers= new ArrayList<>();

        public StructureBuilder addLayer(Layer layer){
            constructionLayers.add(layer);
            return this;
        }

        public Structure build(){
            return new Structure(constructionLayers);
        }

    }

    private Structure (ArrayList<Layer> layers){
        this.layers = layers;
        if (layers.size() != 0){
            currentLayer = 0;
        }
    }

    public int numLayers(){
        return layers.size();
    }

    public Layer currentLayer(){
        if(layers.size() != 0) {
            return layers.get(currentLayer);
        }
        return null;
    }

    public boolean nextLayer(){
        if(layers.size() > currentLayer + 1){
            currentLayer++;
            return true;
        }
        return false;
    }
}
