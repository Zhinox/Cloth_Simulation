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

const int meshRows = 18;
const int meshColumns = 14;
const int totalVertex = meshRows * meshColumns;

static int Ke = 1000;
static float Kd = 50;
static float L = 0.5f;
static float maxElongation = 7.f;
static int resetTime = 10;
static float dtCounter = 0;

glm::vec3 *nodeVectors;
glm::vec3 *lastVectors;
glm::vec3 *velVectors;
glm::vec3 *newVectors;
glm::vec3 *forceVectors;

void GUI() {
	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::SliderInt("Reset Time", &resetTime, 1, 20);
		ImGui::SliderInt("Ke", &Ke, 100, 2000);
		ImGui::SliderFloat("Kd", &Kd, 1, 100);
		ImGui::SliderFloat("Max elongation", &maxElongation, 1, 10);
		ImGui::SliderFloat("Inital rest distance", &L, 0.1f, 0.8f);
	}

	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

glm::vec3 calculateForces(glm::vec3 P1, glm::vec3 P2, glm::vec3 v1, glm::vec3 v2, float Length) {

	float distance = glm::length(P1-P2); //Calculate distance between points

	glm::vec3 velocity = (v1 - v2); //Velocity vector

	glm::vec3 normalVector = glm::normalize(P1 - P2); //Normal vector

	float Calc1 = Ke*(distance - Length) + glm::dot(Kd*velocity,normalVector); //First calculus of force

	glm::vec3 totalResult = (-Calc1) *normalVector; //Final force vector

	/*std::cout << "P1: " << P1.x << " " << P1.y << " " << P1.z << std::endl;
	std::cout << "P2: " << P2.x << " " << P2.y << " " << P2.z << std::endl;
	std::cout << "V1: " << v1.x << " " << v1.y << " " << v1.z << std::endl;
	std::cout << "V2: " << v2.x << " " << v2.y << " " << v2.z << std::endl;
	std::cout << "Distance: " << distance << std::endl;
	std::cout << "Velocity: " << velocity.x << " " << velocity.y << " " << velocity.z << std::endl;
	std::cout << "Normal Vector: " << normalVector.x << " " << normalVector.y << " " << normalVector.z << std::endl;
	std::cout << "Calcul 1: " << Calc1 << std::endl;
	std::cout << "Totalresult: " << totalResult.x << " " << totalResult.y << " " << totalResult.z << std::endl << std::endl;*/

	return totalResult;

}

glm::vec3 calculateAllForces(glm::vec3 vectorsPos[], glm::vec3 vectorsVel[], int calcVector) {

	glm::vec3 totalForces;

	//Structural
	if (calcVector % 18 != 17) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector + 1], vectorsVel[calcVector], vectorsVel[calcVector + 1], L); } //Dreta 

	if (calcVector % 18 != 0) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector - 1], vectorsVel[calcVector], vectorsVel[calcVector - 1], L); }//Esquerra

	if (calcVector / 18 != 0) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector - 18], vectorsVel[calcVector], vectorsVel[calcVector - 18],L); } //Adalt

	if (calcVector / 18 != 13) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector + 18], vectorsVel[calcVector], vectorsVel[calcVector + 18],L); } //Abaix

	//Shear
	if (calcVector % 18 != 17 && calcVector / 18 != 0) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector - 17], vectorsVel[calcVector], vectorsVel[calcVector - 17], sqrt(L*L+L*L)); }//Diagonal dreta adalt

	if (calcVector % 18 != 17 && calcVector / 18 != 13) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector + 19], vectorsVel[calcVector], vectorsVel[calcVector + 19], sqrt(L*L + L*L)); }//Diagonal dreta abaix

	if (calcVector % 18 != 0 && calcVector / 18 != 0) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector + 17], vectorsVel[calcVector], vectorsVel[calcVector + 17], sqrt(L*L + L*L)); }//Diagonal esquerra adalt

	if (calcVector % 18 != 0 && calcVector / 18 != 13) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector - 19], vectorsVel[calcVector], vectorsVel[calcVector - 19], sqrt(L*L + L*L)); } //Diagonal esquerra abaix

	//Bending
	if (calcVector % 18 != 17 && calcVector % 18 != 16) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector + 2], vectorsVel[calcVector], vectorsVel[calcVector + 2], L*2); } //Doble dreta

	if (calcVector % 18 != 0 && calcVector % 18 != 1) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector - 2], vectorsVel[calcVector], vectorsVel[calcVector - 2], L*2); } //Doble esquerra

	if (calcVector / 18 != 0 && calcVector / 18 != 1) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector - 36], vectorsVel[calcVector], vectorsVel[calcVector - 36], L*2); } //Doble adalt

	if (calcVector / 18 != 13 && calcVector / 18 != 12) { totalForces += calculateForces(vectorsPos[calcVector], vectorsPos[calcVector + 36], vectorsVel[calcVector], vectorsVel[calcVector + 36], L*2); } //Doble abaix

	return totalForces;

}

void PhysicsInit() {
	nodeVectors = new glm::vec3[totalVertex];
	velVectors = new glm::vec3[totalVertex];
	newVectors = new glm::vec3[totalVertex];
	forceVectors = new glm::vec3[totalVertex];

	int columnsCounter = 0;
	int rowsCounter = 0;

	for (int i = 0; i < totalVertex; i++) {
		nodeVectors[i] = { L * columnsCounter - (L*meshColumns/2) + L/2,8, L * rowsCounter - (L*meshRows/2) + L/2};
		velVectors[i] = {0,0,0};
		newVectors[i] = nodeVectors[i];
		forceVectors[i] = calculateAllForces(nodeVectors, velVectors, i);
	
		if (columnsCounter >= 13) {
			columnsCounter = 0;
			rowsCounter += 1;
		}
		else {columnsCounter += 1;}
	}	

	
}


void PhysicsUpdate(float dt) {
	
	//Top left always the same positions
	nodeVectors[0] = { L * 0 - (L*meshColumns / 2) + L / 2,8, L * 0 - (L*meshRows / 2) + L / 2 };

	//Top right always the same positions
	nodeVectors[13] = { L * 13 - (L*meshColumns / 2) + L / 2,8, L * 0 - (L*meshRows / 2) + L / 2 };

	for (int i = 0; i < totalVertex; i++) { //Applying physics on all nodes
		

		forceVectors[i] = calculateAllForces(nodeVectors, velVectors, i);
		
		newVectors[i] = nodeVectors[i] + dt * velVectors[i]; //Euler

		velVectors[i].x = velVectors[i].x + dt * 0;
		velVectors[i].y = velVectors[i].y + dt * -9.81;
		velVectors[i].z = velVectors[i].z + dt * 0;

		//std::cout << "Velocity: " << i << " " << velVectors[i].x << " " << velVectors[i].y << " " << velVectors[i].z << std::endl;
		//std::cout << "Position: " << i << " " << newVectors[i].x << " " << newVectors[i].y << " " << newVectors[i].z << std::endl;

		nodeVectors[i] = newVectors[i]; //Update position

	}

	dtCounter += dt;
	if (dtCounter >= resetTime) { PhysicsInit(); dtCounter = 0; } //Reset every "x" seconds

	ClothMesh::updateClothMesh(&nodeVectors[0].x);

}


void PhysicsCleanup() {
	
}