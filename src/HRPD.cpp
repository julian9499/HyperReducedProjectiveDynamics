/*
********************************************************************************
MIT License

Copyright(c) 2018 Christopher Brandt

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
********************************************************************************
*/

/* libIGL include files for loading meshes and the OpenGL viewer */
#include "igl/readOBJ.h"
#include "igl/opengl/glfw/Viewer.h"
#include "igl/opengl/glfw/ViewerPlugin.h"
#include "igl/opengl/glfw/imgui/ImGuiPlugin.h"
#include "igl/opengl/glfw/imgui/ImGuiMenu.h"
#include "igl/opengl/glfw/imgui/ImGuiHelpers.h"

/* HRPD include files */
#include "ProjDynTypeDef.h"
#include "ProjDynSimulator.h"

/* Constructs a simulator object, sets constraints, adds gravity, a floor with friction and
triggers the precomputation. */
ProjDynSimulator* initSimulator(PD::PDPositions verts, PD::PDTriangles faces, PD::PDPositions velos, std::string meshURL) {
	// Construct simulator object from this mesh and provide initial velcocities,
	// as well as a few reduction and simulation parameters, which are explained
	// in the _README.txt
	double timeStep = 16;
	int numberSamplesForVertexPosSubspace = 120; // The number of degrees of freedom for the mesh vertex positions will be 12 times that
	double radiusMultiplierForVertexPosSubspace = 1.1; // The larger this number, the larger the support of the base functions.
	int dimensionOfConstraintProjectionsSubspace = 120; // The constraint projections subspace will be constructed to be twice that size and then condensed via an SVD
	double radiusMultiplierForConstraintProjectionsSubspace = 2.2;
	int numberSampledConstraints = 1000; // Number of constraints that will be evaluated each iteration
	ProjDynSimulator* sim =
		new ProjDynSimulator(faces, verts, velos, timeStep, 
			numberSamplesForVertexPosSubspace, radiusMultiplierForVertexPosSubspace,
			dimensionOfConstraintProjectionsSubspace, radiusMultiplierForConstraintProjectionsSubspace,
			numberSampledConstraints, 
			2, 0, true, meshURL,
			0., 3);


	// We add gravity and floor friction+repulsion to the simulation
	sim->addGravity(0.0001);
	sim->addFloor(1, 0, 1);
	sim->setFrictionCoefficient(0.5, 0.05);


    // For the simulation to be meaningful we need some sorts of constraints
    // The following method adds volume preservation constraints to all tets
    sim->addTetStrain(0.00051, 1.f, 1.f);
//    sim->addEdgeSprings(0.051, 1.f, 1.f);


    // The call to the setup function computes the subspaces for vertex
	// positions and constraint projections and factorizes all linear systems
	// for local and global steps
	sim->setup();

	return sim;
}

/*
This class is an auxilary class to enable pre-draw and key-press events with
the libIGL OpenGL viewer.
Skip to the main() function below first to get a basic idea of this example.
*/
class SimViewer : public igl::opengl::glfw::ViewerPlugin {

public:

	ProjDynSimulator* m_sim = nullptr;
	PDPositions& m_verts;
	PDPositions& m_velos;
	PDPositions m_gripPos;
	PDTriangles& m_faces;
	std::string m_url;
	StopWatch m_simTimer;
	std::vector<unsigned int> m_gripVerts;
    std::vector<int> m_chosenColors;
	bool m_isGripping = false;
	bool m_toggleStiff = false;
    bool m_toggleSlowmo = false;
    bool m_toggleTime = false;
	int m_numIterations;
	int m_numVertices;
    long m_adjacencyTiming;
    long m_coloringTiming;


    SimViewer(PDPositions& verts, PDTriangles& faces, PDPositions& velos, std::string url, int numIterations) :
		m_verts(verts),
		m_faces(faces),
		m_velos(velos),
		m_simTimer(10000, 100000),
		m_url(url),
		m_isGripping(true),
		m_toggleStiff(false)
	{
		m_sim = initSimulator(m_verts, m_faces, m_velos, m_url);
		m_numIterations = numIterations;
		m_numVertices = m_verts.rows();
        m_chosenColors = m_sim->getChosenColors();
        m_adjacencyTiming = m_sim->getAdjacencyTiming();
        m_coloringTiming = m_sim->getColoringTiming();

		/* The following is specific to the armadillo.obj mesh, it's a list of vertices on the
		   back, on which the mesh is "hung up" when the user presses 3.
		   Change accordingly if using other meshes (can also be an emtpy list). */
		std::vector<unsigned int> backVerts { 0, 1, 2, 3, 4, 5 ,6 ,7 };
		m_gripVerts = backVerts;
		m_gripPos.setZero(m_gripVerts.size(), 3);
		for (int i = 0; i < m_gripVerts.size(); i++) {
			m_gripPos.row(i) = verts.row(m_gripVerts[i]);
			m_gripPos(i, 1) += 5.5;
		}
	}

	/*
	Before the mesh is rendered, do a time step of the simulator and hand over the updated
	vertex positions to the viewer.
	Note that you would want to directly map the vertex positions into the OpenGL buffer
	in a high-performance setting, instead of passing them like this.
	*/
	bool pre_draw() {
		if (m_sim) {
			m_simTimer.startStopWatch();
			m_sim->step(m_numIterations);
			m_simTimer.stopStopWatch();
			viewer->data().set_mesh(m_sim->getPositions().block(0,0,m_numVertices,3), m_faces);
			return false;
		}
		return false;
	}

	/*
		Customized key events recognized by the libIGL viewer
	*/
	bool key_pressed(unsigned int key, int modifiers) override
	{
		if (key == 49) {
			// Print FPS
			int avgMs = m_simTimer.evaluateAverage() / 1000;
			std::cout << "Average ms for time-step (includes vertex position updates, but not rendering): " << avgMs << std::endl;
			std::cout << "That is, the simulation runs at " << (1000 / avgMs) << " FPS" << std::endl;
		}
		else if (key == 50) {
			// reset positions
			if (m_sim) {
				m_sim->resetPositions();
			}
			m_isGripping = false;
		}
		else if (key == 51) {
			// hang mesh high up at back vertices or release it (toggle)
			if (m_sim) {
				if (!m_isGripping) {
					m_sim->setGrip(m_gripVerts, m_gripPos);
					m_isGripping = true;
				}
				else {
					m_sim->releaseGrip();
					m_isGripping = false;
				}
			}
		}
		else if (key == 52) {
			// toggle stiffness between normal and super elastic
			if (m_sim) {
				if (!m_toggleStiff) {
					m_sim->setStiffnessFactor(0.2);
					m_toggleStiff = true;
				}
				else {
					m_sim->setStiffnessFactor(1);
					m_toggleStiff = false;
				}
			}
		}
		else if (key == 53) {
			// toggle between bullet time and real time
			if (m_sim) {
				if (!m_toggleSlowmo) {
					m_sim->changeTimeStep(6);
					m_toggleSlowmo = true;
				}
				else {
					m_sim->changeTimeStep(16);
					m_toggleSlowmo = false;
				}
			}
		}
		else if (key == 54) {
			// Reset simulation (i.e. complete recomputation)
			if (m_sim) {
				delete m_sim;
				m_sim = initSimulator(m_verts, m_faces, m_velos, m_url);
			}
			m_isGripping = false;
			m_toggleStiff = false;
			m_toggleSlowmo = false;
		}
		return false;
	}


};

/*
Here we load a mesh, initialize the simulation and a viewer and start the main
render/timestep loop.
*/
int main()
{
	// Depending on whatever your default working directory is and wherever this mesh
	// file is, you will need to change this URL
    std::string meshURL = "armadillo.obj";
//    std::string meshURL = "sphere.obj";

	// Load a mesh using IGL
	PD::PDPositions verts, velos;
	PD::PDTriangles faces;
	if (igl::readOBJ(meshURL, verts, faces)) {
		int numVertices = verts.rows();
		velos.setZero(numVertices, 3);

		// Number of local/global iterations in the reduced projective dynamics solver
		int numIterations = 10;

		// Start a viewer that uses a simple plugin to draw the simulated mesh
		// (it does numIterations local/global steps to simulate the next timestep
		// before drawing the mesh).

		SimViewer simViewer(verts, faces, velos, meshURL, numIterations);
        std::vector<int> sorted = simViewer.m_chosenColors;
        std::sort(sorted.begin(), sorted.end());
        std::cout << sorted[sorted.size()-1] + 1 << std::endl;
        std::cout << std::endl;
        int amountOfColors = sorted[sorted.size()-1] + 1;
        Eigen::MatrixXd C;
        C.resize( verts.rows(), 3);
        Eigen::MatrixXd pointColors;
        pointColors.resize( simViewer.m_numVertices, 3);
        Eigen::MatrixXd chosenPoints;
        std::vector<std::vector<double>> rainbow_colors {
                { 0.8588235294117647, 0.5803921568627451, 0.20784313725490197},
                { 0.3411764705882353, 0.40784313725490196, 0.8352941176470589},
                { 0.5294117647058824, 0.7333333333333333, 0.21568627450980393},
                { 0.5686274509803921, 0.37254901960784315, 0.8392156862745098},
                { 0.30196078431372547, 0.7764705882352941, 0.36470588235294116},
                { 0.6823529411764706, 0.25098039215686274, 0.7019607843137254},
                { 0.2627450980392157, 0.5607843137254902, 0.17254901960784313},
                { 0.8862745098039215, 0.43137254901960786, 0.8313725490196079},
                { 0.5137254901960784, 0.7411764705882353, 0.4196078431372549},
                { 0.8823529411764706, 0.24313725490196078, 0.5098039215686274},
                { 0.3764705882352941, 0.796078431372549, 0.6274509803921569},
                { 0.7294117647058823, 0.23529411764705882, 0.5372549019607843},
                { 0.28627450980392155, 0.6235294117647059, 0.3764705882352941},
                { 0.5372549019607843, 0.3137254901960784, 0.6274509803921569},
                { 0.7529411764705882, 0.6980392156862745, 0.2235294117647059},
                { 0.3803921568627451, 0.5607843137254902, 0.9019607843137255},
                { 0.8392156862745098, 0.32941176470588235, 0.13333333333333333},
                { 0.25098039215686274, 0.7647058823529411, 0.8313725490196079},
                { 0.803921568627451, 0.2235294117647059, 0.22745098039215686},
                { 0.2196078431372549, 0.611764705882353, 0.5176470588235295},
                { 0.8196078431372549, 0.27058823529411763, 0.37254901960784315},
                { 0.20784313725490197, 0.4549019607843137, 0.23137254901960785},
                { 0.6784313725490196, 0.5607843137254902, 0.8666666666666667},
                { 0.5215686274509804, 0.5764705882352941, 0.19215686274509805},
                { 0.3254901960784314, 0.39215686274509803, 0.6470588235294118},
                { 0.36470588235294116, 0.4235294117647059, 0.09803921568627451},
                { 0.8392156862745098, 0.5411764705882353, 0.7725490196078432},
                { 0.15294117647058825, 0.4470588235294118, 0.3411764705882353},
                { 0.8588235294117647, 0.43529411764705883, 0.611764705882353},
                { 0.39215686274509803, 0.43529411764705883, 0.21176470588235294},
                { 0.38823529411764707, 0.615686274509804, 0.8274509803921568},
                { 0.8980392156862745, 0.44313725490196076, 0.3333333333333333},
                { 0.6862745098039216, 0.6901960784313725, 0.4196078431372549},
                { 0.592156862745098, 0.32941176470588235, 0.5137254901960784},
                { 0.5647058823529412, 0.4392156862745098, 0.1803921568627451},
                { 0.6039215686274509, 0.25098039215686274, 0.34901960784313724},
                { 0.8627450980392157, 0.6039215686274509, 0.4235294117647059},
                { 0.7058823529411765, 0.3568627450980392, 0.3411764705882353},
                { 0.6352941176470588, 0.34509803921568627, 0.16862745098039217},
                { 0.9137254901960784, 0.5529411764705883, 0.5803921568627451},
        };
        
        std::vector<std::vector<double>> hsl_colors = {
                {1.0, 0.25098039215686274, 0.25098039215686274},
                        {1.0, 0.792156862745098, 0.7490196078431373},
                        {0.2, 0.10196078431372549, 0.0},
                        {0.8980392156862745, 0.5607843137254902, 0.2235294117647059},
                        {0.45098039215686275, 0.3764705882352941, 0.0},
                        {0.7019607843137254, 0.6705882352941176, 0.5254901960784314},
                        {0.8980392156862745, 0.9019607843137255, 0.2235294117647059},
                        {0.0, 0.7019607843137254, 0.11764705882352941},
                        {0.45098039215686275, 0.9019607843137255, 0.6745098039215687},
                        {0.08627450980392157, 0.34901960784313724, 0.2627450980392157},
                        {0.45098039215686275, 0.8235294117647058, 0.9019607843137255},
                        {0.050980392156862744, 0.12549019607843137, 0.2},
                        {0.34901960784313724, 0.4666666666666667, 0.7019607843137254},
                        {0.23921568627450981, 0.23921568627450981, 0.9490196078431372},
                        {0.30196078431372547, 0.10196078431372549, 0.4},
                        {1.0, 0.7490196078431373, 0.9568627450980393},
                        {0.8509803921568627, 0.21176470588235294, 0.6392156862745098},
                        {0.45098039215686275, 0.0, 0.14901960784313725}
        };
        std::vector<std::vector<double>>& used_color = rainbow_colors;

        for(int i = 0; i < pointColors.rows(); i++) {
            int chosenColor = simViewer.m_chosenColors[i];
            if (i < verts.rows()) {
                C(i, 0) = used_color[chosenColor][0];
                C(i, 1) = used_color[chosenColor][1];
                C(i, 2) = used_color[chosenColor][2];
            }
            pointColors(i, 0) = used_color[chosenColor][0];
            pointColors(i, 1) = used_color[chosenColor][1];
            pointColors(i, 2) = used_color[chosenColor][2];
        }



		igl::opengl::glfw::Viewer viewer;
        igl::opengl::glfw::imgui::ImGuiPlugin plugin;
        viewer.plugins.push_back(&plugin);
        igl::opengl::glfw::imgui::ImGuiMenu menu;
        plugin.widgets.push_back(&menu);
        float r = 0.5;
        float g = 0.5;
        float b = 0.0;

		viewer.data().set_mesh(verts, faces);
        viewer.data().uniform_colors(Eigen::Vector3d(r,g,b), Eigen::Vector3d(r,g,b), Eigen::Vector3d(r,g,b));
        viewer.core().is_animating = true;
        viewer.data().set_colors(C);
		viewer.plugins.push_back(&simViewer);
        // Add content to the default menu window

        bool color_verts = true;
        bool color_points = false;
        bool draw_selected_color = false;

        menu.callback_draw_viewer_menu = [&]()
        {
            // Draw parent menu content
            menu.draw_viewer_menu();

            // Add new group
            if (ImGui::CollapsingHeader("Change coloring", ImGuiTreeNodeFlags_DefaultOpen))
            {

                ImGui::Checkbox("color vertices", &color_verts);
                ImGui::Checkbox("color points", &color_points);
                ImGui::Checkbox("Only draw selected color", &draw_selected_color);

                // We can also use a std::vector<std::string> defined dynamically
                static std::vector<std::string> choices;
                static int selectedColor = 0;

                if (amountOfColors != (int) choices.size())
                {
                    choices.resize(amountOfColors);
                    for (int i = 0; i < amountOfColors; ++i)
                        choices[i] = std::to_string(i);;
                    if (selectedColor >= amountOfColors)
                        selectedColor = amountOfColors - 1;
                }
                ImGui::Combo("selectedColor", &selectedColor, choices);


                // Expose an enumeration type
                enum ColorPattern { Rainbow=0, HSL };
                static ColorPattern color = Rainbow;
                ImGui::Combo("Color pattern", (int *)(&color), "Rainbow\0Hsl\0");
                // Add a button
                if (ImGui::Button("set Color", ImVec2(-1,0)))
                {
                    switch (color) {
                        case Rainbow:
                            used_color = rainbow_colors;
                            break;
                        case HSL:
                            used_color = hsl_colors;
                            break;
                    }


                    int count = std::count(simViewer.m_chosenColors.begin(), simViewer.m_chosenColors.end(), selectedColor);
                    if  (draw_selected_color){
                        chosenPoints.resize( count, 3);
                    } else {
                        chosenPoints.resize( pointColors.rows(), 3);
                    }
                    count = 0;
                    for(int i = 0; i < pointColors.rows(); i++) {
                        int chosenColor = simViewer.m_chosenColors[i];

                        if ((draw_selected_color && chosenColor == selectedColor) || !draw_selected_color) {
                            if (i < verts.rows()){
                                C(i, 0) = used_color[chosenColor][0];
                                C(i, 1) = used_color[chosenColor][1];
                                C(i, 2) = used_color[chosenColor][2];
                            }
                            pointColors(i, 0) = used_color[chosenColor][0];
                            pointColors(i, 1) = used_color[chosenColor][1];
                            pointColors(i, 2) = used_color[chosenColor][2];
                            chosenPoints.row(count) = simViewer.m_sim->getPositions().row(i);
                            count += 1;
                        } else {
                            if (i < verts.rows()){
                                C(i, 0) = 1.0;
                                C(i, 1) = 1.0;
                                C(i, 2) = 0.0;
                            }
                        }
                    }

                    viewer.data().clear_points();
//                    viewer.data().clear();
                    viewer.data().uniform_colors(Eigen::Vector3d(r,g,b), Eigen::Vector3d(r,g,b), Eigen::Vector3d(0.f,0.f,0.f));

                    viewer.data().set_mesh(simViewer.m_sim->getPositions().block(0,0,numVertices,3), faces);
                    if (color_verts){
                        viewer.data().set_colors(C);
                    }
                    if (color_points && draw_selected_color) {
                        viewer.data().add_points(chosenPoints, Eigen::RowVector3d(used_color[selectedColor][0],used_color[selectedColor][1],used_color[selectedColor][2]));
                    } else if (color_points) {
                        viewer.data().add_points(chosenPoints, pointColors);
                    }
                }




            }
        };
        // Draw additional windows
        menu.callback_draw_custom_window = [&]()
        {
            // Define next window position + size
            ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), true);
            ImGui::SetNextWindowSize(ImVec2(300, 100), true);
            ImGui::Begin(
                    "statistics", nullptr,
                    ImGuiWindowFlags_NoSavedSettings
            );

            ImGui::Text("Time taken to build adjacency matrix (ms): %ld", simViewer.m_adjacencyTiming);
            ImGui::Text("Time taken to color the graph (ms): %ld", simViewer.m_coloringTiming);
            ImGui::Text("Amount of colors: %d", amountOfColors);

            ImGui::End();
        };
		viewer.init_plugins();
		viewer.launch();
	}
	else {
		std::cout << "Could not load mesh! Put the mesh file into the working directory (VS project directory or next to the HRPD.exe)." << std::endl;
		system("pause");
		return -1;
	}

    return 0;
}

