package fr.unice.polytech.deantoni.vrep.polybot.utils;

public final class NodeData<T> { 

    private final T nodeId;

    private double g;  // g is distance from the source
    private double h = 1;  // h is the heuristic of destination.
    private double f;  // f = g + h 

    public NodeData (T nodeId) {
        this.nodeId = nodeId;
        this.g = Double.MAX_VALUE; 
    }

    public T getNodeId() {
        return nodeId;
    }

    public double getG() {
        return g;
    }

    public void setG(double g) {
        this.g = g;
    }

    public void calcF(T destination) {
        this.f = g + h;
    } 

    public double getH() {
        return h;
    }

    public double getF() {
        return f;
    }
 }


