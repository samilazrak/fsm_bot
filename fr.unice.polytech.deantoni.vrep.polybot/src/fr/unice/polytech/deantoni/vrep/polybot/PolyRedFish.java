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

import fr.unice.polytech.deantoni.vrep.polybot.robot.PolyRob;
import fr.unice.polytech.deantoni.vrep.polybot.utils.Blob;
import fr.unice.polytech.deantoni.vrep.polybot.utils.Position2D;

/**
 * Random walk of the rob to illustrate the use of the PolyRob API. the goal of this class
 * is to illustrate an unstructured code that control a robot in V-REP. It is
 * used in Finite State Machine Course where the student have to rewrite the
 * control code in a state chart. The associated hands on is located
 * here: @http://www.i3s.unice.fr/~deantoni/teaching_resources/SI4/FSM/TDs/
 * 
 * @author Julien Deantoni
 */
public class PolyRedFish {

	public static void main(String[] args) {
		PolyRob rob = new PolyRob("127.0.0.1", 19997);
		redFishBehaviour(rob);
		// dumbRedFishStuff(rob);
	}

	protected static void redFishBehaviour(PolyRob rob) {
		rob.start();
		rob.sleep(800);
		int i = 0;
		while (i < 5) {
			i++;
			double dist = 100;
			while (!rob.hasDetectedAnObject() || dist > 75) {
				dist = rob.getDetectedObjectDistance();
				ArrayList<Blob> blobs = rob.getViewableBlobs();
				for (Blob b : blobs) {
					System.out.println("\t blob [" + b.positionX + " ; " + b.positionY + "]");
				}
				Position2D pos = rob.getPosition();
				System.out.println("rob [" + pos.x + " ; " + pos.y + "]");
				int sl = (int) (Math.random() * 15);
				int sr = (int) (Math.random() * 15);
				rob.goCurved(sl, sr);
				rob.sleep(100);
			}

			rob.goStraight(0);
			dist = rob.getDetectedObjectDistance();
			rob.log2vrep("dist " + dist);

			while (dist > 55) {

				System.out.println(dist + "\n\t" + rob.detectedObjectPoint.getArray()[0] + " "
						+ rob.detectedObjectPoint.getArray()[1] + " " + rob.detectedObjectPoint.getArray()[2]);
				rob.goStraight(2);
				if (rob.detectedObjectPoint.getArray()[0] > 0) {
					rob.goCurved(2, 3);
					System.out.println("turn left");
				} else {
					rob.goCurved(3, 2);
					System.out.println("turn right");
				}
				rob.log2vrep("dist " + dist);
				rob.sleep(100);
				rob.hasDetectedAnObject();
				dist = rob.detectedObjectPoint.getArray()[0] * rob.detectedObjectPoint.getArray()[0]
						+ rob.detectedObjectPoint.getArray()[1] * rob.detectedObjectPoint.getArray()[1]
								+ rob.detectedObjectPoint.getArray()[2] * rob.detectedObjectPoint.getArray()[2];
			}
			rob.log2vrep("last dist " + dist);

			rob.closeGrip();
			rob.sleep(1500);
			rob.goStraight(-4, 500);
			rob.turnLeft(7, 2000);
			rob.goStraight(15);
			rob.sleep(4000);
			rob.openGrip();
			rob.goStraight(-5, 3000);
			rob.turnLeft(6, 1000);
			rob.sleep(1500);
		}
		rob.stopSimulation();
	}

	protected static void dumbRedFishStuff(PolyRob rob) {
		int i = 0;
		rob.start();
		while (i++ < 10) {
			if (i % 2 == 0) {
				rob.openGrip();
			} else {
				rob.closeGrip();
			}
			rob.goStraight(5);
			while (!rob.hasDetectedAnObject()) {
				rob.sleep(100);
			}
			rob.goStraight(-6);
			rob.sleep(400);
			if (rob.detectedObjectPoint.getArray()[2] > rob.detectedObjectPoint.getArray()[0]) {
				rob.turnRight(6);
				rob.sleep(600);
			} else {
				rob.turnRight(6);
				rob.sleep(600);
			}
		}
		rob.stopSimulation();
	}

}
