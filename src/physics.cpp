#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <iostream>

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

float *meshArray;

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

	for (int i = 0; i < meshRows; i++) {
		for (int j = 0; j < meshColumns; j++) {

			meshArray[i * meshColumns * 3 + j * 3 + 0] = -2 + 4 * j / (float)meshColumns;
			meshArray[i * meshColumns * 3 + j * 3 + 1] = 7 - 4 * i / (float)meshRows;
			meshArray[i * meshColumns * 3 + j * 3 + 2] = 0;

			std::cout << i << " " << j << std::endl;
		}
		std::cout << "i++" << std::endl;
		
	}
}
void PhysicsUpdate(float dt) {
	
	ClothMesh::updateClothMesh(meshArray);
}
void PhysicsCleanup() {
	
}