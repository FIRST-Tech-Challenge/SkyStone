package org.firstinspires.ftc.teamcode.auto.structurebuilder;

import java.util.ArrayList;

public class Layer {

    private ArrayList<Stone> stones;
    private int currentStone;

    public static class LayerBuilder {

        ArrayList<Stone> constructionStones = new ArrayList<>();

        public LayerBuilder addStone(Stone stone){
            if(isValid(stone)){
                constructionStones.add(stone);
            }
            return this;
        }

        private boolean isValid(Stone stone){
            for(Stone s : constructionStones){
                if(s.getOccupiedSpace()[0].getX() == stone.getOccupiedSpace()[0].getX() || s.getOccupiedSpace()[0].getX() == stone.getOccupiedSpace()[1].getX() ||
                        s.getOccupiedSpace()[1].getX() == stone.getOccupiedSpace()[0].getX() || s.getOccupiedSpace()[1].getX() == stone.getOccupiedSpace()[1].getX() ||
                        s.getOccupiedSpace()[0].getY() == stone.getOccupiedSpace()[0].getY() || s.getOccupiedSpace()[0].getY() == stone.getOccupiedSpace()[1].getY() ||
                        s.getOccupiedSpace()[1].getY() == stone.getOccupiedSpace()[0].getY() || s.getOccupiedSpace()[1].getY() == stone.getOccupiedSpace()[1].getY()){
                    return false;
                }
            }
            return true;
        }

        public Layer build(){
            return new Layer(constructionStones);
        }
    }

    private Layer(ArrayList<Stone> stones){
        this.stones = stones;
        if(stones.size() != 0){
            currentStone = 0;
        }
    }

    public Stone getCurrentStone(){
        return stones.get(currentStone);
    }

    public boolean nextStone(){
        if(stones.size() > currentStone + 1){
            currentStone++;
            return true;
        }
        return false;
    }

}
