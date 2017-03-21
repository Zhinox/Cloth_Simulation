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

struct meshNodes {
	glm::vec3 vector = { 0,0,0 };
	glm::vec3 velvector = { 0,0,0 };
	float totalForce = 9;
};


const int meshRows = 18;
const int meshColumns = 14;
const int totalVertex = meshRows * meshColumns;

float *meshArray;
glm::vec3 *nodeVectors;

void GUI() {
	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		
	}

	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void PhysicsInit() {
	meshArray = new float[totalVertex * 3];
	nodeVectors = new glm::vec3[totalVertex];

	int columnsCounter = 0;
	int rowsCounter = 0;
	for (int i = 0; i < totalVertex; i++) {
		nodeVectors[i] = { -3.5f + 7.5f * columnsCounter / (float)meshColumns,8,-3.5f + 7.5f * rowsCounter / (float)meshRows };

		if (columnsCounter >= 13) {
			columnsCounter = 0;
			rowsCounter += 1;
		}
		else {columnsCounter += 1;}
	}	

}
void PhysicsUpdate(float dt) {
	
	//Top left always the same positions
	nodeVectors[0] = { -3.5f + 7.5f * 0 / (float)meshColumns ,8,-3.5f + 7.5f * 0 / (float)meshColumns };

	//Top right always the same positions
	nodeVectors[13] = { -3.5f + 7.5f * 13 / (float)meshColumns ,8,-3.5f + 7.5f * 0 / (float)meshColumns };

	ClothMesh::updateClothMesh(&nodeVectors[0].x);

}
void PhysicsCleanup() {
	
}