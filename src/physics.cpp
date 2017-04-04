#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <GL\glew.h>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <cstdio>
#include <iostream>
#include <time.h>
#include <math.h>

bool show_test_window = false;

namespace ClothMesh {
	void setupClothMesh();
	void cleanupClothMesh();
	void updateClothMesh(float *array_data);
	void drawClothMesh();
};

//Mesh variables
const int meshRows = 18;
const int meshColumns = 14;
const int totalVertex = meshRows * meshColumns;

static int Ke = 100; //Stiffness
static float Kd = 0.5; //Damping
static float L = 0.5f;
static float maxElongation = 7.f;
static int resetTime = 10;
static float dtCounter = 0;

static int lastKe;
static float lastKd, lastL, lastElongation, lastTime;

//Mesh arrays
glm::vec3 *nodeVectors;
glm::vec3 *lastVectors;
glm::vec3 *velVectors;
glm::vec3 *newVectors;
glm::vec3 *forceVectors;

//Cube planes
glm::vec3 terraN = { 0,1,0 };
glm::vec3 sostreN = { 0,-1,0 };
glm::vec3 leftN = { 1,0,0 };
glm::vec3 rightN = { -1,0,0 };
glm::vec3 backN = { 0,0,1 };
glm::vec3 frontN = { 0,0,-1 };

void GUI() {

	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::SliderInt("Reset Time", &resetTime, 0, 20);
	ImGui::SliderInt("Ke", &Ke, 100, 2000);
	ImGui::SliderFloat("Kd", &Kd, 1, 100);
	ImGui::SliderFloat("Max elongation", &maxElongation, 1, 10);
	ImGui::SliderFloat("Inital rest distance", &L, 0.1f, 0.75f);

	if (show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

float calculateCollision(glm::vec3 vector, glm::vec3 lastvector, glm::vec3 normal, int d) {

	float calc1 = glm::dot(vector, normal) + d;
	float calc2 = glm::dot(lastvector, normal) + d;
	return calc1*calc2;

}

glm::vec3 calculateAllCollisions(glm::vec3 vectorPos[], glm::vec3 vectorsVel[], glm::vec3 lastPosition, int mode, int calcVector) {

	if (calculateCollision(vectorPos[calcVector], lastPosition, terraN, 0) <= 0) { //Collision with ground
		if (mode == 1) { return vectorPos[calcVector] - (1 + 0.9f) * (glm::dot(terraN, vectorPos[calcVector]) + 0) * terraN; }
		else { return vectorsVel[calcVector] - (1 + 0.9f) * (glm::dot(terraN, vectorsVel[calcVector]) + 0) * terraN; }
	}

	if (calculateCollision(vectorPos[calcVector], lastPosition, sostreN, 10) <= 0) { //Collision with roof
		if (mode == 1) { return vectorPos[calcVector] - (1 + 0.9f) * (glm::dot(sostreN, vectorPos[calcVector]) + 10) * sostreN; }
		else { return vectorsVel[calcVector] - (1 + 0.9f) * (glm::dot(sostreN, vectorsVel[calcVector]) + 10) * sostreN; }
	}

	if (calculateCollision(vectorPos[calcVector], lastPosition, leftN, 5) <= 0) { //Collision with left wall
		if (mode == 1) { return vectorPos[calcVector] - (1 + 0.9f) * (glm::dot(leftN, vectorPos[calcVector]) + 5) * leftN; }
		else { return vectorsVel[calcVector] - (1 + 0.9f) * (glm::dot(leftN, vectorsVel[calcVector]) + 5) * leftN; }
	}

	if (calculateCollision(vectorPos[calcVector], lastPosition, rightN, 5) <= 0) { //Collision with right wall
		if (mode == 1) { return vectorPos[calcVector] - (1 + 0.9f) * (glm::dot(rightN, vectorPos[calcVector]) + 5) * rightN; }
		else { return vectorsVel[calcVector] - (1 + 0.9f) * (glm::dot(rightN, vectorsVel[calcVector]) + 5) * rightN; }
	}

	if (calculateCollision(vectorPos[calcVector], lastPosition, frontN, 5) <= 0) { //Collision with front wall
		if (mode == 1) { return vectorPos[calcVector] - (1 + 0.9f) * (glm::dot(frontN, vectorPos[calcVector]) + 5) * frontN; }
		else { return vectorsVel[calcVector] - (1 + 0.9f) * (glm::dot(frontN, vectorsVel[calcVector]) + 5) * frontN; }
	}

	if (calculateCollision(vectorPos[calcVector], lastPosition, backN, 5) <= 0) { //Collision with back wall
		if (mode == 1) { return vectorPos[calcVector] - (1 + 0.9f) * (glm::dot(backN, vectorPos[calcVector]) + 5) * backN; }
		else { return vectorsVel[calcVector] - (1 + 0.9f) * (glm::dot(backN, vectorsVel[calcVector]) + 5) * backN; }
	}

	else {
		if (mode == 1) { return vectorPos[calcVector]; } //If no collision is detected, return the same vector
		else { return vectorsVel[calcVector]; }
	}
}


glm::vec3 calculateForces(glm::vec3 P1, glm::vec3 P2, glm::vec3 v1, glm::vec3 v2, float Length) {

	float distance = glm::length(P1 - P2); //Calculate distance between points

	glm::vec3 velocity = (v1 - v2); //Velocity vector

	glm::vec3 normalVector = glm::normalize(P1 - P2); //Normal vector

	float Calc1 = Ke*(distance - Length) + glm::dot(Kd*velocity, normalVector); //First calculus of force

	glm::vec3 totalResult = (-Calc1) *normalVector; //Final force vector

	return totalResult;

}

glm::vec3 calculateAllForces(glm::vec3 vectorsPos[], glm::vec3 vectorsVel[], int calcVector) {

	glm::vec3 totalForces;

	//Structural
	if (calcVector % 14 != 13) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector + 1], vectorsVel[calcVector], vectorsVel[calcVector + 1], L); } //Dreta 

	if (calcVector % 14 != 0) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector - 1], vectorsVel[calcVector], vectorsVel[calcVector - 1], L); }//Esquerra

	if (calcVector / 14 != 0) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector - 14], vectorsVel[calcVector], vectorsVel[calcVector - 14], L); } //Adalt

	if (calcVector / 14 != 17) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector + 14], vectorsVel[calcVector], vectorsVel[calcVector + 14], L); } //Abaix

																																												//Shear
	if (calcVector % 14 != 13 && calcVector / 14 != 0) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector - 13], vectorsVel[calcVector], vectorsVel[calcVector - 13], sqrt(L*L + L*L)); } //Diagonal dreta adalt

	if (calcVector % 14 != 13 && calcVector / 14 != 17) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector + 15], vectorsVel[calcVector], vectorsVel[calcVector + 15], sqrt(L*L + L*L)); } //Diagonal dreta abaix

	if (calcVector % 14 != 0 && calcVector / 14 != 0) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector - 15], vectorsVel[calcVector], vectorsVel[calcVector - 15], sqrt(L*L + L*L)); } //Diagonal esquerra adalt

	if (calcVector % 14 != 0 && calcVector / 14 != 17) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector + 13], vectorsVel[calcVector], vectorsVel[calcVector + 13], sqrt(L*L + L*L)); } //Diagonal esquerra abaix

																																																					  //Bending
	if (calcVector % 14 != 13 && calcVector % 14 != 12) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector + 2], vectorsVel[calcVector], vectorsVel[calcVector + 2], L * 2); } //Doble dreta

	if (calcVector % 14 != 0 && calcVector % 14 != 1) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector - 2], vectorsVel[calcVector], vectorsVel[calcVector - 2], L * 2); } //Doble esquerra

	if (calcVector / 14 != 0 && calcVector / 14 != 1) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector - 28], vectorsVel[calcVector], vectorsVel[calcVector - 28], L * 2); } //Doble adalt

	if (calcVector / 14 != 17 && calcVector / 14 != 16) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector + 28], vectorsVel[calcVector], vectorsVel[calcVector + 28], L * 2); } //Doble abaix

	return totalForces;

}

void reset() {

	//Function that resets to the beggining all the positions, forces and velocites of the mesh
	int columnsCounter = 0;
	int rowsCounter = 0;

	for (int i = 0; i < totalVertex; i++) {
		nodeVectors[i] = { L * columnsCounter - (L*meshColumns / 2) + L / 2,8, L * rowsCounter - (L*meshRows / 2) + L / 2 };
		velVectors[i] = { 0,0,0 };
		newVectors[i] = nodeVectors[i];
		forceVectors[i] = { 0,0,0 };
		if (columnsCounter >= 13) {
			columnsCounter = 0;
			rowsCounter += 1;
		}
		else { columnsCounter += 1; }
	}
}

void checkChanges() {

	//Check for changes on variables to reset the Mesh
	if (lastKe != Ke || lastKd != Kd || lastElongation != maxElongation || lastL != L || lastTime != resetTime) {
		lastKe = Ke;
		lastKd = Kd;
		lastElongation = maxElongation;
		lastL = L;
		lastTime = resetTime;
		reset();
	}
}

void PhysicsInit() {

	//Creation of all glm::vec3 arrays
	nodeVectors = new glm::vec3[totalVertex];
	velVectors = new glm::vec3[totalVertex];
	newVectors = new glm::vec3[totalVertex];
	forceVectors = new glm::vec3[totalVertex];
	lastVectors = new glm::vec3[totalVertex];

	//Declaration of mesh counters
	int columnsCounter = 0;
	int rowsCounter = 0;

	//Applying values to "last" variables for reseting
	lastKe = Ke;
	lastKd = Kd;
	lastElongation = maxElongation;
	lastL = L;
	lastTime = resetTime;

	//For that creates the Mesh with an "L" separation
	for (int i = 0; i < totalVertex; i++) {
		nodeVectors[i] = { L * columnsCounter - (L*meshColumns / 2) + L / 2,8, L * rowsCounter - (L*meshRows / 2) + L / 2 };
		velVectors[i] = { 0,0,0 };
		newVectors[i] = nodeVectors[i];
		forceVectors[i] = { 0,0,0 };
		if (columnsCounter >= 13) {
			columnsCounter = 0;
			rowsCounter += 1;
		}
		else { columnsCounter += 1; }
	}

}


void PhysicsUpdate(float dt) {

	//Top left always the same positions
	nodeVectors[0] = { L * 0 - (L*meshColumns / 2) + L / 2,8, L * 0 - (L*meshRows / 2) + L / 2 };

	//Top right always the same positions
	nodeVectors[13] = { L * 13 - (L*meshColumns / 2) + L / 2,8, L * 0 - (L*meshRows / 2) + L / 2 };

	for (int i = 0; i < totalVertex; i++) { //Applying forces and velocities on all nodes

		if (i == 0 || i == 13) { forceVectors[i] = { 0,0,0 }; }
		else {

			forceVectors[i] = calculateAllForces(nodeVectors, velVectors, i); //Calculate forces and store them on array

			lastVectors[i] = newVectors[i]; //Store last position vector

											//Velocities and gravity
			velVectors[i].x = velVectors[i].x + dt * forceVectors[i].x;
			velVectors[i].y = velVectors[i].y + dt * (-9.81f + forceVectors[i].y);
			velVectors[i].z = velVectors[i].z + dt * forceVectors[i].z;

			forceVectors[i] = glm::vec3(0, 0, 0); //Reset forces
		}
	}

	for (int i = 0; i < totalVertex; i++) { //Applying Euler's solver and upating

		newVectors[i] = nodeVectors[i] + dt * velVectors[i]; //Euler 

		nodeVectors[i] = newVectors[i]; //Update position
	}

	dtCounter += dt;

	checkChanges();

	if (dtCounter >= resetTime) { reset(); dtCounter = 0; } //Reset every "x" seconds

	ClothMesh::updateClothMesh(&nodeVectors[0].x);

}


void PhysicsCleanup() {

}