/*******************************************************************************
 * Copyright (c) 2018 I3S laboratory and others.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *     Julien Deantoni - initial API and implementation
 *******************************************************************************/

package fr.unice.polytech.deantoni.vrep.polybot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import fr.unice.polytech.deantoni.vrep.polybot.robot.PolyRob;
import fr.unice.polytech.deantoni.vrep.polybot.utils.AStar;
import fr.unice.polytech.deantoni.vrep.polybot.utils.Blob;
import fr.unice.polytech.deantoni.vrep.polybot.utils.GraphAStar;
import fr.unice.polytech.deantoni.vrep.polybot.utils.NodeData;
import fr.unice.polytech.deantoni.vrep.polybot.utils.Position2D;

/**
 * not smart enough but better than the Red fish version. the goal of this class
 * is to illustrate an unstructured code that control a robot in V-REP. It is
 * used in Finite State Machine Course where the student have to rewrite the
 * control code in a state chart. The associated hands on is located
 * here: @http://www.i3s.unice.fr/~deantoni/teaching_resources/SI4/FSM/TDs/
 * 
 * @author Julien Deantoni
 */
public class PolyBrain extends PolyRob{
	
	/**
	 * a Map to store interesting things, like walls, etc
	 */
	protected ArrayList<Position2D> flatMap = new ArrayList<Position2D>(mapFactor*mapFactor);
	protected GraphAStar<Position2D> graph = new GraphAStar<Position2D>();
	//may need an obstacle map for soft reset
	protected List<Position2D> path;
	protected int indexInPath;
	
	protected Position2D safePlace = new Position2D(-45, 500);
	
	protected boolean pathToCompute = true;
	protected boolean goToSafePlace = false;
	protected boolean isInSafePlace = false;
	protected boolean closeMode = false;
	
	public PolyBrain(String IP, int portNumber) {
		super(IP, portNumber);
		initializeMap();
	}
	
	protected void initializeMap() {    
		graph = new GraphAStar<Position2D>();
		flatMap.clear();
		for(int i=0; i < mapFactor; i++) {
        	for(int j = 0; j < mapFactor; j++) {
        		Position2D p = new Position2D(i, j);
        		graph.addNode(p);
        		flatMap.add(p);
        	}
        }
        for(int i=0; i < mapFactor; i++) {
        	for(int j = 0; j < mapFactor; j++) {
        		setNeighbourHoodCost(graph, i, j, 1);
        	}
        }
	}

	public static void main(String[] args) {
		PolyBrain rob = new PolyBrain("127.0.0.1", 19997);
		rob.start();
		rob.realizeMission();
	}

	public void realizeMission() {
		this.openGrip();
		this.readNoseSensor();
		ArrayList<Blob> blobs = this.getViewableBlobs();
		while(!blobs.isEmpty() || (this.closeMode  || this.goToSafePlace || this.isInSafePlace)) {
			System.out.println("still paint bombs to move...");
			Position2D blobPos = new Position2D(blobs.get(0).positionX, blobs.get(0).positionY);
			Position2D robPos = this.getPosition();
			float robOrientation = this.getOrientation();
			this.closeMode = this.updateMapAndMode(blobs, robPos, robOrientation);
			if(this.closeMode){
				this.smoothMoveToTarget();
			}else if (this.goToSafePlace) {
				this.goToSafePlace(robPos, robOrientation);
			}else if (this.isInSafePlace){
				this.openGrip();
				this.sleep(600);
				this.goStraight(-7,2500);
				this.isInSafePlace = false;
			}else {
				blobs = this.getViewableBlobs();
		        Position2D destination = this.computeNextDestination(blobPos, robPos, true);
			    this.goToDestination(blobPos, robPos, robOrientation, destination);
			}
		}
		System.out.println("job finished...");
		this.goStraight(0);
	}

	protected boolean updateMapAndMode(ArrayList<Blob> blobs, Position2D robPos, float robOrientation) {
		if(this.hasDetectedAnObject()) {
			int objDist = this.getDetectedObjectDistance();
			System.out.println("object distance: "+objDist+ "("+robOrientation+") --> "+robPos.x+","+robPos.y);

			Position2D objectCoordinate = new Position2D(
												(int)Math.round(robPos.x+Math.cos(robOrientation)*objDist),
												(int)Math.round(robPos.y+Math.sin(robOrientation)*objDist)
										  );
			boolean addIt = true;
			
			addIt = checkIfDetectedObjectIsAPaintBomb(blobs, objectCoordinate, addIt);
			if (addIt) {
				pathToCompute = true;
				boundCoordinate(objectCoordinate);
				System.out.println("add obstacle "+objectCoordinate.x+","+objectCoordinate.y);
				this.modifyNeighbourHoodCost(this.graph, objectCoordinate.x, objectCoordinate.y, 200);
			}
		}
		return closeMode;
	}
	
	protected void setNeighbourHoodCost(GraphAStar<Position2D> graph, int i, int j, int cost) {
		if (i > 1)  graph.addEdge(flatMap.get((i*mapFactor)+j), flatMap.get(((i-1)*mapFactor)+j), cost);
		if (j < (mapFactor-1)) graph.addEdge(flatMap.get((i*mapFactor)+j), flatMap.get((i*mapFactor)+j+1), cost);
		if (j > 0)  graph.addEdge(flatMap.get((i*mapFactor)+j), flatMap.get((i*mapFactor)+j-1), cost);
		if (i < (mapFactor-1)) graph.addEdge(flatMap.get((i*mapFactor)+j), flatMap.get(((i+1)*mapFactor)+j), cost);
	}

	protected void modifyNeighbourHoodCost(GraphAStar<Position2D> graph, int i, int j, int cost) {
		Map<NodeData<Position2D>, Double> edgesToChange = graph.edgesFrom(flatMap.get((i*mapFactor)+j));
		for (Entry<NodeData<Position2D>, Double> edgeEntry : edgesToChange.entrySet()) {
			edgeEntry.setValue((double)cost);
		}
	}
	
	protected boolean checkIfDetectedObjectIsAPaintBomb(ArrayList<Blob> blobs, Position2D objectCoordinate,boolean addIt) {
		for(Blob b: blobs) {
			if ((Math.abs(b.positionX - objectCoordinate.x) <= 50) && (Math.abs(b.positionY - objectCoordinate.y) <= 50)) {
				System.out.println("detected object is a paint bomb");
				closeMode = true;
				addIt = false;
			}
		}
		return addIt;
	}

	protected void boundCoordinate(Position2D objectCoordinate) {
		if (objectCoordinate.x < 0) objectCoordinate.x = 0;
		if (objectCoordinate.x >= mapFactor) objectCoordinate.x = mapFactor-1;
		if (objectCoordinate.y < 0) objectCoordinate.y = 0;
		if (objectCoordinate.y >= mapFactor) objectCoordinate.y = mapFactor-1;
	}

	protected void smoothMoveToTarget() {
		if (!this.hasDetectedAnObject()) {
			this.turnRight(3);
			return;
		}
		int dist = this.getDetectedObjectDistance();
		System.out.println("distance =" +dist);
		if (dist == 100) {
			this.turnLeft(2);
			return;
		}
		if(dist > 60) {
			this.goStraight(3);
			if (this.detectedObjectPoint.getArray()[0] > 0){ //turn left
				this.goCurved(3,4);
			}
			else{ //turn right
				this.goCurved(4,3);
			}
			this.sleep(100);
			this.hasDetectedAnObject();
		}else {
			this.goStraight(3, 300);
			this.closeGrip();
			this.goStraight(3,1500);
			closeMode = false;
			goToSafePlace  = true;
		}
	}

	protected Position2D computeNextDestination(Position2D blobPos, Position2D robPos, boolean doCloseMode) {
		Position2D destination = null;
		if (pathToCompute) {
			indexInPath = 20;
			AStar<Position2D> aStar = new AStar<Position2D>(this.graph);
			boundCoordinate(robPos);
			path = aStar.astar(this.flatMap.get(this.mapFactor*robPos.x+robPos.y), this.flatMap.get(this.mapFactor*blobPos.x+blobPos.y));
			if (path != null && path.size() < 20 && doCloseMode) {
				closeMode = true;
			}
		}
		if (path == null) {
			System.err.println("no path to get the paint bomb..");
			this.initializeMap(); //try to reinitialize everything
			destination = blobPos;
		}else {
			pathToCompute = false;
			System.out.print(indexInPath+ "# ");
			path.forEach(p -> System.out.print(p.x+","+p.y+ " | "));
			System.out.println("__");
			indexInPath +=25;
			if(path.size() - indexInPath < 40) {
				pathToCompute = true;
			}
		    destination = path.get(indexInPath);
		}
		return destination;
	}

	protected void goToDestination(Position2D blobPos, Position2D robPos, float robOrientation,Position2D destination) {
		//are we close enough ?
		if ((Math.abs(blobPos.x - robPos.x) <= 60) && (Math.abs(blobPos.y - robPos.y) <= 60)) {
			System.out.println("here we are close enought of a paint bomb "+blobPos.x+","+blobPos.y);
			System.out.println("\trob: "+robPos.x + ","+robPos.y+ "("+robOrientation+")");
			this.closeMode = true;
		}
		
		double angleToReach = computeAngleToReach(robPos, destination); //in [0; 2pi[
		double angleToDo = robOrientation-angleToReach;
		if (angleToDo > Math.PI) {
			angleToDo = angleToDo - 2*Math.PI ; 
		}
		System.out.println("-----------------------------------------------");
		System.out.println("rob: "+robPos.x + ","+robPos.y+ "("+robOrientation+")");
		System.out.println("dest: "+destination.x + ","+destination.y);
		System.out.println("blob: "+blobPos.x + ","+blobPos.y);
		System.out.println("angle to reach: "+angleToReach);
		System.out.println("angle to do : "+angleToDo);
		
		if(angleToDo > 0) { this.turnRight(4); this.sleep((int)Math.abs(angleToDo*1300));} //black magic computation
		else {this.turnLeft(4); this.sleep((int)Math.abs(angleToDo*1300));}
		
		this.goStraight(8); //should depends on the destination distance...
		this.sleep(300);
	}
	
	protected void goToSafePlace(Position2D robPos, float robOrientation) {
		if ((Math.abs(safePlace.x - robPos.x) <= 10) && (Math.abs(safePlace.y - robPos.y) <= 10)) {
			System.out.println("here we are close enought to the safe place "+safePlace.x+","+safePlace.y);
			System.out.println("\trob: "+robPos.x + ","+robPos.y+ "("+robOrientation+")");
			this.goToSafePlace = false;
			this.isInSafePlace = true;
		}
		
		double angleToReach = computeAngleToReach(robPos, safePlace); //in [0; 2pi[
		double angleToDo = robOrientation-angleToReach;
		if (angleToDo > Math.PI) {
			angleToDo = angleToDo - 2*Math.PI ; 
		}
		System.out.println("-----------------------------------------------");
		System.out.println("rob: "+robPos.x + ","+robPos.y+ "("+robOrientation+")");
		System.out.println("dest: "+safePlace.x + ","+safePlace.y);
		System.out.println("angle to reach: "+angleToReach);
		System.out.println("angle to do : "+angleToDo);
		
		if(angleToDo > 0) { this.turnRight(4); this.sleep((int)Math.abs(angleToDo*1300));}
		else {this.turnLeft(4); this.sleep((int)Math.abs(angleToDo*1300));}
		
		this.goStraight(5); //should depends on the destination distance...
		this.sleep(300);
	}

	protected double computeAngleToReach(Position2D robPos, Position2D destination) {
		double angleToReach = 0;
		angleToReach =Math.atan2(destination.y -robPos.y, destination.x - robPos.x);
		return angleToReach;
	}

}
