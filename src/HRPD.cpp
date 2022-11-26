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
	int numberSamplesForVertexPosSubspace = 0;//150; // The number of degrees of freedom for the mesh vertex positions will be 12 times that
	double radiusMultiplierForVertexPosSubspace = 1.1; // The larger this number, the larger the support of the base functions.
	int dimensionOfConstraintProjectionsSubspace = 120; // The constraint projections subspace will be constructed to be twice that size and then condensed via an SVD
	double radiusMultiplierForConstraintProjectionsSubspace = 2.2;
	int numberSampledConstraints = 0;//1000; // Number of constraints that will be evaluated each iteration
	ProjDynSimulator* sim =
		new ProjDynSimulator(faces, verts, velos, timeStep, 
			numberSamplesForVertexPosSubspace, radiusMultiplierForVertexPosSubspace,
			dimensionOfConstraintProjectionsSubspace, radiusMultiplierForConstraintProjectionsSubspace,
			numberSampledConstraints, 
			2, 0, true, meshURL,
			0., 3);

	// For the simulation to be meaningful we need some sorts of constraints
	// The following method adds volume preservation constraints to all tets
//    sim->addTetStrain(0.00051, 1.f, 1.f);
    sim->addEdgeSprings(0.051, 1.f, 1.f);

	// We add gravity and floor friction+repulsion to the simulation
	sim->addGravity(0.0001);
	sim->addFloor(1, -2, 1);
	sim->setFrictionCoefficient(0.5, 0.05);

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
	int m_numIterations;
	int m_numVertices;

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
	bool key_pressed(unsigned int key, int modifiers)
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
		return true;
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
	std::string meshURL = "sheet.obj";

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
        std::cout << sorted[sorted.size()-1] << std::endl;
        std::cout << simViewer.m_chosenColors.size() << std::endl;
        std::cout << simViewer.m_verts.rows() << std::endl;
        Eigen::MatrixXd C;
        C.resize( simViewer.m_verts.rows(), 3);
        std::vector<std::vector<double>> colors {
                {105.0/255.0, 105.0/255.0, 105.0/255.0},
                {211.0/255.0, 211.0/255.0, 211.0/255.0},
                {46.0/255.0, 139.0/255.0, 87/255.0},
                {139.0/255.0, 0.0, 0.0},
                {128.0/255.0, 128.0/255.0, 0.0},
                {0.0, 0.0, 139.0/255.0},
                {1.0, 0.0, 0.0},
                {1.0, 165.0/255.0, 0.0},
                {1.0, 1.0, 0.0},
                {127.0/255.0, 1.0, 0.0},
                {0.0, 250.0/255.0, 154.0/255.0},
                {65.0/255.0, 105.0/255.0, 225.0/255.0},
                {0.0, 1.0, 1.0},
                {0.0, 191.0/255.0, 1.0},
                {0.0, 0.0, 1.0},
                {218.0/255.0, 112.0/255.0, 214.0/255.0},
                {1.0, 0.0, 1.0},
                {240.0/255.0, 230.0/255.0, 140.0/255.0},
                {1.0, 20.0/255.0, 147.0/255.0},
                {1.0, 20.0/255.0, 147.0/255.0},
                {1.0, 20.0/255.0, 147.0/255.0},
                {1.0, 20.0/255.0, 147.0/255.0},
                {1.0, 20.0/255.0, 147.0/255.0}
        };

        for(int i = 0; i < simViewer.m_verts.rows(); i++) {
            int chosenColor = simViewer.m_chosenColors[i];
            C(i, 0) = colors[chosenColor][0];
            C(i, 1) = colors[chosenColor][1];
            C(i, 2) = colors[chosenColor][2];
        }

        std::cout << verts.rows() << std::endl;


		igl::opengl::glfw::Viewer viewer;
		viewer.data().set_mesh(verts, faces);
        viewer.data().set_colors(C);
		viewer.core().is_animating = true;
		viewer.plugins.push_back(&simViewer);
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

