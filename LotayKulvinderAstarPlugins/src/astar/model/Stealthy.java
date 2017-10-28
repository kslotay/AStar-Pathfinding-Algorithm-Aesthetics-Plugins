/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package astar.model;

import astar.plugin.IModel;
import astar.util.Node;

/**
 *
 * @author Kulvinder
 */
public class Stealthy implements IModel {
    protected char[][] tileMap = null;

    @Override
    public void init(char[][] tileMap) {
        this.tileMap = tileMap;
    }

    @Override
    public double shape(double heuristic, Node curNode, Node adjNode) {
        /*
            Inertia is a calculation of the second derivative of the gradient,
            i.e the change in the slope of the path.
        
            This is a metric that can be used to optimize for continuous paths
            rather than zig-zags.
        
            The inertia is inversely proportional to the cost, such that
            a higher inertia means a lower cost.
        
            Additionally, if the second derivative is zero, then the
            movement cost is discounted
        */
        double cost = 0;
        
        
        if(curNode.getParent() != null){
            Node parentNode = curNode.getParent();
            
            int inertia = 1;
            
            Node adjHolder = curNode;
            Node curHolder = parentNode;
            Node parentHolder = parentNode.getParent();

            // check for zags and tracks
            boolean columnCheck = ((parentNode.getCol() - curNode.getCol()) != (curNode.getCol() - adjNode.getCol()));
            boolean rowCheck = ((parentNode.getRow() - curNode.getRow()) != (curNode.getRow() - adjNode.getRow()));
            
            if((columnCheck || rowCheck) && (astar.util.Helper.tracksWall(this.tileMap, adjNode))){
                cost -= 5;
            }
            
            // test the weight between two nodes
            else{
                if(astar.util.Helper.tracksWall(this.tileMap, adjNode)){
                    // weight of .5 for a track
                    cost += 0;
                }

                // check for zig zag
                if( columnCheck || rowCheck){
                    cost += 7;
                }
            }

            boolean colSlope = ((parentNode.getCol() - curNode.getCol()) == ((curNode.getCol()) - adjNode.getCol()));
            boolean rowSlope = ((parentNode.getRow() - curNode.getRow()) == (curNode.getRow() - adjNode.getRow()));  
            
            while((colSlope && rowSlope) && (inertia >=0)){
                // keep checking the parent node to determine whether to add more momentum
                // we will keep receiving a parent node until we reach a different slope or until we reach the end of the list
                
                inertia -= .15;
                
                if(parentHolder == null){
                    break;
                }
                
                colSlope = ((parentHolder.getCol() - curHolder.getCol()) == ((curHolder.getCol()) - adjHolder.getCol()));
                rowSlope = ((parentHolder.getRow() - curHolder.getRow()) == (curHolder.getRow() - adjHolder.getRow()));
                adjHolder = adjHolder.getParent();
                curHolder = curHolder.getParent();
                parentHolder = parentHolder.getParent();

            }
            
            // add inertia to cost
            cost += inertia;
            
            
        }
        return cost;
    }

    @Override
    public void complete(Node curNode) {
    }
    
}
