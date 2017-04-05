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
static float elasticity = 0.8f;
static int maxElongation = 7;
static int resetTime = 10;
static float dtCounter = 0;

static int lastKe, lastElongation;
static float lastKd, lastL,lastTime;

//Mesh arrays
glm::vec3 *nodeVectors;
glm::vec3 *lastVectors;
glm::vec3 *velVectors;
glm::vec3 *newVectors;
glm::vec3 *forceVectors;

//Cube planes
glm::vec3 groundN = { 0,1,0 };
glm::vec3 roofN = { 0,-1,0 };
glm::vec3 leftN = { 1,0,0 };
glm::vec3 rightN = { -1,0,0 };
glm::vec3 backN = { 0,0,1 };
glm::vec3 frontN = { 0,0,-1 };

void GUI() {

	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::SliderInt("Reset Time", &resetTime, 0, 20);
	ImGui::SliderInt("Ke", &Ke, 100, 2000);
	ImGui::SliderFloat("Kd", &Kd, 1, 100);
	ImGui::SliderInt("Max elongation (%)", &maxElongation, 1, 100);
	ImGui::SliderFloat("Inital rest distance", &L, 0.1f, 0.75f);
	ImGui::SliderFloat("Elasticity", &elasticity, 0.1f, 0.9f);

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

void calculateAllCollisions(glm::vec3 &vectorPos, glm::vec3 &vectorsVel, glm::vec3 &lastPosition) {

	if (calculateCollision(vectorPos, lastPosition, groundN, 0) < 0) { //Collision with ground
		vectorPos = vectorPos - (1 + elasticity) * (glm::dot(groundN, vectorPos) + 0) * groundN; 
		vectorsVel = vectorsVel - (1 + elasticity) * (glm::dot(groundN, vectorsVel) + 0) * groundN;
	}

	if (calculateCollision(vectorPos, lastPosition, roofN, 10) <= 0) { //Collision with roof
		vectorPos = vectorPos - (1 + elasticity) * (glm::dot(roofN, vectorPos) + 10) * roofN; 
		vectorsVel = vectorsVel - (1 + elasticity) * (glm::dot(roofN, vectorsVel) + 0) * roofN; 
	}

	if (calculateCollision(vectorPos, lastPosition, leftN, 5) <= 0) { //Collision with left wall
		vectorPos = vectorPos - (1 + elasticity) * (glm::dot(leftN, vectorPos) + 5) * leftN; 
		vectorsVel = vectorsVel - (1 + elasticity) * (glm::dot(leftN, vectorsVel) + 0) * leftN; 
	}

	if (calculateCollision(vectorPos, lastPosition, rightN, 5) <= 0) { //Collision with right wall
		vectorPos = vectorPos - (1 + elasticity) * (glm::dot(rightN, vectorPos) + 5) * rightN; 
		vectorsVel = vectorsVel - (1 + elasticity) * (glm::dot(rightN, vectorsVel) + 0) * rightN; 
	}

	if (calculateCollision(vectorPos, lastPosition, frontN, 5) <= 0) { //Collision with front wall
		vectorPos = vectorPos - (1 + elasticity) * (glm::dot(frontN, vectorPos) + 5) * frontN; 
		vectorsVel = vectorsVel - (1 + elasticity) * (glm::dot(frontN, vectorsVel) + 0) * frontN; 
	}

	if (calculateCollision(vectorPos, lastPosition, backN, 5) <= 0) { //Collision with back wall
		vectorPos = vectorPos - (1 + elasticity) * (glm::dot(backN, vectorPos) + 5) * backN; 
		vectorsVel = vectorsVel - (1 + elasticity) * (glm::dot(backN, vectorsVel) + 0) * backN; 
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

void checkElongation(glm::vec3 posVectors[]) {

	float distanceRight = 0;
	float distanceDown = 0;
	glm::vec3 unitariRight;
	glm::vec3 unitariDown;

	float maxL = (L * maxElongation) / 100;

	for (int i = 0; i < totalVertex; i++) {
		
		if (i % 14 != 13 && i / 14 != 17) {

			distanceRight = glm::length(posVectors[i] - posVectors[i + 1]);
			distanceDown = glm::length(posVectors[i] - posVectors[i + 14]);
			unitariRight = glm::normalize(posVectors[i] - posVectors[i + 1]);
			unitariDown = glm::normalize(posVectors[i] - posVectors[i + 14]);

			if (distanceRight > maxL) {
				posVectors[i] += (maxL / 2) * unitariRight;
				posVectors[i + 1] -= (maxL / 2) * unitariRight;

			}

			if (distanceDown > maxL) {
				posVectors[i] += (maxL / 2) * unitariDown;
				posVectors[i + 14] -= (maxL / 2) * unitariDown;
			}
		}
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
		}
	}

	for (int i = 0; i < totalVertex; i++) { //Applying Euler's solver and upating

		if (i == 0 || i == 13) { forceVectors[i] = { 0,0,0 }; }
		else {

		lastVectors[i] = nodeVectors[i]; //Store last position vector

		//Velocities and gravity
		velVectors[i].x = velVectors[i].x + dt * forceVectors[i].x;
		velVectors[i].y = velVectors[i].y + dt * (-9.81f + forceVectors[i].y);
		velVectors[i].z = velVectors[i].z + dt * forceVectors[i].z;

		forceVectors[i] = glm::vec3(0, 0, 0); //Reset forces
		
		newVectors[i] = nodeVectors[i] + dt * velVectors[i]; //Euler 

		nodeVectors[i] = newVectors[i]; //Update position

		calculateAllCollisions(nodeVectors[i], velVectors[i], lastVectors[i]);

		checkElongation(nodeVectors);
		
		}
		
	}

	dtCounter += dt;

	checkChanges();

	if (dtCounter >= resetTime) { reset(); dtCounter = 0; } //Reset every "x" seconds

	ClothMesh::updateClothMesh(&nodeVectors[0].x);

}


void PhysicsCleanup() {

}