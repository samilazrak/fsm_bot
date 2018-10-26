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
package fr.unice.polytech.deantoni.vrep.polybot.robot;

import java.util.ArrayList;

import coppelia.BoolW;
import coppelia.FloatWA;
import coppelia.FloatWAA;
import coppelia.IntW;
import coppelia.remoteApi;
import fr.unice.polytech.deantoni.vrep.polybot.utils.Blob;
import fr.unice.polytech.deantoni.vrep.polybot.utils.Position2D;

public class PolyRob {

	protected int clientID = -1;
	protected remoteApi vrep = new remoteApi();

	// handler of robot equipments
	protected IntW rightGrip = new IntW(0);
	protected IntW rightGripBis = new IntW(0);
	protected IntW leftMotor = new IntW(0);
	protected IntW rightMotor = new IntW(0);
	protected IntW proxSensor = new IntW(0);
	protected IntW camera = new IntW(0);
	protected IntW rob = new IntW(0);

	// handlers on detected objects
	protected BoolW objectDetected = new BoolW(false);

	/**
	 * x, y and z of the closest detected point
	 */
	public FloatWA detectedObjectPoint = new FloatWA(0);
	public IntW handleDetectedObj = new IntW(0);
	public FloatWA mapDetectedObject = new FloatWA(0);

	// robot parameters
	protected float gripForce = (float) 0.08;
	protected int mapFactor = 1000;

	public PolyRob(String IP, int portNumber) {
		clientID = vrep.simxStart(IP, portNumber, true, true, 5000, 5);
		if (clientID == -1) {
			throw new RuntimeException("impossible to connect to V-REP server");
		} else {
			System.out.println("connected to the server....");
		}
		// Handle rob !
		vrep.simxGetObjectHandle(clientID, "PolyRob", rob, remoteApi.simx_opmode_blocking);
		// Handle of the grip
		vrep.simxGetObjectHandle(clientID, "PolyRobOpenCloseJoint", rightGrip, remoteApi.simx_opmode_blocking);
		// Handle of the grip bis
		vrep.simxGetObjectHandle(clientID, "PolyRobOpenCloseJoint0", rightGripBis, remoteApi.simx_opmode_blocking);
		// Handle of the left motor
		vrep.simxGetObjectHandle(clientID, "PolyRobLeftMotor", leftMotor, remoteApi.simx_opmode_blocking);
		// Handle of the right motor
		vrep.simxGetObjectHandle(clientID, "PolyRobRightMotor", rightMotor, remoteApi.simx_opmode_blocking);
		// Handle of the proximity sensor
		vrep.simxGetObjectHandle(clientID, "PolyRobSensingNose", proxSensor, remoteApi.simx_opmode_blocking);
		// Handle of the camera sensor
		vrep.simxGetObjectHandle(clientID, "blobDetectionCamera_camera", camera, remoteApi.simx_opmode_blocking);

		System.out.println("ready to go ! ");
	}

	public void start() {
		vrep.simxStartSimulation(clientID, remoteApi.simx_opmode_oneshot);
	}

	public void readNoseSensor() {
		vrep.simxReadProximitySensor(clientID, proxSensor.getValue(), objectDetected, detectedObjectPoint,
				handleDetectedObj, mapDetectedObject, remoteApi.simx_opmode_blocking);
		return;
	}

	/**
	 * 
	 * @return true if an object is close detected. Detected object is store in
	 *         {@link #handleDetectedObj} and sensor values in #detectedObjectPoint.
	 *         A detected Surface Normal Vector is also store in
	 *         {@link #mapDetectedObject}
	 */
	public boolean hasDetectedAnObject() {
		vrep.simxReadProximitySensor(clientID, proxSensor.getValue(), objectDetected, detectedObjectPoint,
				handleDetectedObj, mapDetectedObject, remoteApi.simx_opmode_blocking);
		return objectDetected.getValue();
	}

	public int getDetectedObjectDistance() {
		double distX = detectedObjectPoint.getArray()[0];
		double distY = detectedObjectPoint.getArray()[1];
		return (int) (Math.sqrt(distX * distX + distY * distY) * mapFactor * 10);
	}

	public void openGrip() {
		vrep.simxSetJointTargetVelocity(clientID, rightGrip.getValue(), gripForce, remoteApi.simx_opmode_streaming);
		vrep.simxSetJointTargetVelocity(clientID, rightGripBis.getValue(), gripForce, remoteApi.simx_opmode_streaming);
	}

	public void closeGrip() {
		vrep.simxSetJointTargetVelocity(clientID, rightGrip.getValue(), -gripForce, remoteApi.simx_opmode_streaming);
		vrep.simxSetJointTargetVelocity(clientID, rightGripBis.getValue(), -gripForce, remoteApi.simx_opmode_streaming);
	}

	/**
	 * make the robot go forward straight if @speed is positive and backward
	 * if @speed is negative
	 * 
	 * @param speed:
	 *            the desired speed (0 = stopped, negative = backward, positive =
	 *            forward)
	 */
	public void goStraight(int speed) {
		vrep.simxSetJointTargetVelocity(clientID, leftMotor.getValue(), (float) speed, remoteApi.simx_opmode_streaming);
		vrep.simxSetJointTargetVelocity(clientID, rightMotor.getValue(), (float) speed,
				remoteApi.simx_opmode_streaming);
	}

	/**
	 * make the robot go forward straight if @speed is positive and backward
	 * if @speed is negative
	 * 
	 * @param speed:
	 *            the desired speed (0 = stopped, negative = backward, positive =
	 *            forward)
	 * @param duration:
	 *            the duration, in milliseconds during which the robot goes straight
	 *            and then stop
	 */
	public void goStraight(int speed, int duration) {
		goStraight(speed);
		sleep(duration);
		goStraight(0);
	}

	public void turnRight(int speed) {
		vrep.simxSetJointTargetVelocity(clientID, leftMotor.getValue(), (float) speed, remoteApi.simx_opmode_streaming);
		vrep.simxSetJointTargetVelocity(clientID, rightMotor.getValue(), (float) 0.0, remoteApi.simx_opmode_streaming);
	}

	public void turnRight(int speed, int duration) {
		turnRight(speed);
		sleep(duration);
		turnRight(0);
	}

	public void turnLeft(int speed) {
		vrep.simxSetJointTargetVelocity(clientID, leftMotor.getValue(), (float) 0.0, remoteApi.simx_opmode_streaming);
		vrep.simxSetJointTargetVelocity(clientID, rightMotor.getValue(), (float) speed,
				remoteApi.simx_opmode_streaming);
	}

	public void turnLeft(int speed, int duration) {
		turnLeft(speed);
		sleep(duration);
		turnLeft(0);
	}

	public void goCurved(int speedLeft, int speedRight) {
		vrep.simxSetJointTargetVelocity(clientID, leftMotor.getValue(), (float) speedLeft,
				remoteApi.simx_opmode_streaming);
		vrep.simxSetJointTargetVelocity(clientID, rightMotor.getValue(), (float) speedRight,
				remoteApi.simx_opmode_streaming);
	}

	public void goCurved(int speedLeft, int speedRight, int duration) {
		goCurved(speedLeft, speedRight);
		sleep(duration);
		goStraight(0);
	}

	public void stopSimulation() {
		// Before closing the connection to V-REP, make sure that the last command sent
		// out had time to arrive. You can guarantee this with (for example):
		IntW pingTime = new IntW(0);
		vrep.simxGetPingTime(clientID, pingTime);

		// Now close the connection to V-REP:
		vrep.simxStopSimulation(clientID, remoteApi.simx_opmode_blocking);
		vrep.simxFinish(clientID);
	}

	public Position2D getPosition() {
		FloatWA pos = new FloatWA(3);
		vrep.simxGetObjectPosition(clientID, rob.getValue(), camera.getValue(), pos, remoteApi.simx_opmode_blocking);
		float x = pos.getArray()[0] + 0.40f;
		float y = pos.getArray()[1] - 0.042f;
		Position2D res = new Position2D(0, 0);
		res.y = (int) Math.round(((2.3 + -y) / 4.6) * mapFactor);
		res.x = (int) Math.round(((2.3 + x) / 4.6) * mapFactor);
		return res;
	}

	/**
	 * 
	 * @return the orientation in radian on the z axis in [0;2pi[
	 */
	public float getOrientation() {
		FloatWA angles = new FloatWA(3);
		vrep.simxGetObjectOrientation(clientID, rob.getValue(), camera.getValue(), angles,
				remoteApi.simx_opmode_blocking);
		// Normalization [0; 2pi[
		if (angles.getArray()[2] < 0) {
			return (float) (Math.PI + (Math.PI + angles.getArray()[2]));
		} else {
			return (float) (angles.getArray()[2]);
		}
	}

	public ArrayList<Blob> getViewableBlobs() {
		ArrayList<Blob> res = new ArrayList<Blob>();
		BoolW isThereADetection = new BoolW(false);
		FloatWAA blobInformations = new FloatWAA(20 * 1024 * 1024);
		vrep.simxReadVisionSensor(clientID, camera.getValue(), isThereADetection, blobInformations,
				remoteApi.simx_opmode_blocking); // Here we read the image processing camera!
		if (blobInformations.getArray().length > 0 && blobInformations.getArray()[1] != null
				&& blobInformations.getArray()[1].getLength() > 0) {
			// in t1 we should have the blob information if the camera was set-up correctly
			float blobCount = blobInformations.getArray()[1].getArray()[0];
			float dataSizePerBlob = blobInformations.getArray()[1].getArray()[1];
			// Now we go through all blobs:
			for (int i = 0; i < blobCount; i++) {
				Blob aBlob = new Blob();
				aBlob.size = blobInformations.getArray()[1].getArray()[(int) (2 + (i) * dataSizePerBlob + 0)];
				aBlob.orientation = blobInformations.getArray()[1].getArray()[(int) (2 + (i) * dataSizePerBlob + 1)];
				aBlob.positionX = (mapFactor - Math.round(
						blobInformations.getArray()[1].getArray()[(int) (2 + (i) * dataSizePerBlob + 2)] * mapFactor))
						+ 40;
				aBlob.positionY = mapFactor - Math.round(
						blobInformations.getArray()[1].getArray()[(int) (2 + (i) * dataSizePerBlob + 3)] * mapFactor);
				aBlob.dimension[0] = blobInformations.getArray()[1].getArray()[(int) (2 + (i) * dataSizePerBlob + 4)];
				aBlob.dimension[1] = blobInformations.getArray()[1].getArray()[(int) (2 + (i) * dataSizePerBlob + 5)];

				if (aBlob.dimension[0] > aBlob.dimension[1]) {
					float tmp = aBlob.dimension[0];
					aBlob.dimension[0] = aBlob.dimension[1];
					aBlob.dimension[1] = tmp;
					aBlob.orientation = (float) (aBlob.orientation + Math.PI / 2);
				}

				if (Math.abs(aBlob.dimension[0] - aBlob.dimension[1]) < 0.1) { // ball or square
					if (((aBlob.dimension[0] - 0.023) > 0) && (aBlob.dimension[0] - 0.023) < 0.01) { // a paint bomb
						res.add(aBlob);
					}
				}
			}
		}
		return res;

	}

	public void log2vrep(String s) {
		vrep.simxAddStatusbarMessage(clientID, s, remoteApi.simx_opmode_oneshot);
	}

	public void sleep(int ms) {
		try {
			Thread.sleep(ms);
		} catch (InterruptedException ex) {
			Thread.currentThread().interrupt();
		}
	}
}
