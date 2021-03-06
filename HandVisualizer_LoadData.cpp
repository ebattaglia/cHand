//==============================================================================
/*
	Software License Agreement (BSD License)
	Copyright (c) 2003-2016, CHAI3D.
	(www.chai3d.org)

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	* Redistributions of source code must retain the above copyright
	notice, this list of conditions and the following disclaimer.

	* Redistributions in binary form must reproduce the above
	copyright notice, this list of conditions and the following
	disclaimer in the documentation and/or other materials provided
	with the distribution.

	* Neither the name of CHAI3D nor the names of its contributors may
	be used to endorse or promote products derived from this software
	without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.

	\author    <http://www.chai3d.org>
	\author    Edoardo Battaglia
	\version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "cHand.h"
#include <vector>
#include <math.h>
//------------------------------------------------------------------------------
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl2.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
#include <fstream>
#include <iterator> 
//------------------------------------------------------------------------------


using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
	C_STEREO_DISABLED:            Stereo is disabled
	C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
	C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
	C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

bool savemode = false;

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------
string foldername = "data/";
string filename = "hand-2_session-1_recording-4.dat";

int toskip = 10;
int start_step = 0;

static int choosedata = 0;

std::vector<vector<double>> jointangles_data_mat;

int posecount = 0;
int totalsteps = 0;
bool playdata = FALSE;
bool dataloaded = FALSE;


// Pi
double Pi = 3.14159;

std::vector<double> vecstart{
	0 * 30 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), -0 * 10 * (Pi / 180), -0 * 10 * (Pi / 180),
	0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
	0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
	0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
	0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180)
};

std::vector<std::vector<cVector3d>> fingertransl =
{
	/********************************************************/
	// Thumb
	{
	cVector3d(-0.060f, 0.040f, -0.0f),
	cVector3d(0.00f, 0.0f, 0.0f),
	cVector3d(0.00f, 0.0f, 0.0f),
	cVector3d(0.00f, 0.0f, 0.04622f),
	cVector3d(0.03157f, 0.0f, 0.0f),
	},
	/********************************************************/
	// Index
	{
	cVector3d(0.0f, 0.025f, 0.0f),
	cVector3d(0.00f, 0.0f, 0.0f),
	cVector3d(0.00f, 0.0f, 0.0f),
	cVector3d(0.0f, 0.0f, 0.03978f),
	cVector3d(0.02238f, 0.0f, 0.0f)
	},
	/********************************************************/
	// Middle
	{
	cVector3d(0.0f, 0.0f, 0.0f),
	cVector3d(0.00f, 0.0f, 0.0f),
	cVector3d(0.00f, 0.0f, 0.0f),
	cVector3d(0.0f, 0.0f, 0.04463f),
	cVector3d(0.02633f, 0.0f, 0.0f)
	},
	/********************************************************/
	// Ring
	{
	cVector3d(0.0f, -0.025f, 0.0f),
	cVector3d(0.00f, 0.0f, 0.0f),
	cVector3d(0.00f, 0.0f, 0.0f),
	cVector3d(0.0f, 0.0f, 0.04137f),
	cVector3d(0.02565f, 0.0f, 0.0f)
	},
	/********************************************************/
	// Little
	{
	cVector3d(0.0f, -0.05f, 0.0f),
	cVector3d(0.00f, 0.0f, 0.0f),
	cVector3d(0.00f, 0.0f, 0.0f),
	cVector3d(0.0f, 0.0f, 0.03274f),
	cVector3d(0.01811f, 0.0f, 0.0f)
	}
};

// a world that contains all objects of the virtual environment
cWorld* world;

//  Generic objects used as pivot for camera support (used to rotate the camera around the center)
cGenericObject* center;
// this will stay attached to the hand and is used to show the base reference frame
cGenericObject* center0;

// a camera to render the world in the window display
cCamera* camera;
cTransform cameraZeroTransform;

// Light sources
cDirectionalLight* light1;
cDirectionalLight* light2;

/***************************************************************/
/* Needed for cHand */
// a hand object
cHand* HandModel;

// vectors to store joint angles
std::vector<double> vec;
std::vector<double> vecbuffer;

// GUI
std::vector<string> inputname; // used by the GUI to name each input (joint angle)
static float window_scale = 1.0f; // global window scale
/***************************************************************/

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a handle to window display context
GLFWwindow* window = NULL;

// a level widget to display error
cLevel* levelError;
double maxerror = 0;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback for when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback for when an error with GLFW occurs
void errorCallback(int error, const char* a_description);

// callback for when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// callback for when the mouse is scrolled
void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

// this function renders the scene
void updateGraphics(void);

// this function closes the application
void close(void);

// function to load pose information from a .dat file
vector<vector<double>> loadDatFile(string filename);

// function to calculate rmse
double rmse_from_vec(std::vector<double>, std::vector<double>);

/* Debug utilities */
void printcTransform(cTransform T, string Tname);

//==============================================================================
/*
	DEMO:   HandVisualizer_LoadData.cpp

	This application loads data from a file and plays it on the Tkach hand model.

	In this example the application opens an OpenGL window and displays a
	3D hand model, with a simple GUI to load data and play it as a video.
	The GUI is built using Dear ImGUI and all components needed for the GUI are
	included in this folder.

	Keys W,A,S,D are used to rotate the camera, while the mouse scroll wheel can
	be used to zoom in and out. Key V can be used to toggle axes visualization
	on and off, F to toggle visualization for the base reference frame.
*/
//==============================================================================

int main(int argc, char* argv[])
{
	//--------------------------------------------------------------------------
	// INITIALIZATION
	//--------------------------------------------------------------------------
	cout << endl;
	cout << "-----------------------------------" << endl;
	cout << "CHAI3D" << endl;
	cout << "Demo: HandVisualizer" << endl;
	cout << "-----------------------------------" << endl << endl << endl;
	cout << "Keyboard Options:" << endl << endl;
	cout << "[v] - Toggle joint axes" << endl;
	cout << "[w] - Rotate camera - up" << endl;
	cout << "[s] - Rotate camera - down" << endl;
	cout << "[a] - Rotate camera - left" << endl;
	cout << "[d] - Rotate camera - right" << endl;
	cout << "[f] - Show/Hide reference frame" << endl;
	cout << "[q] - Exit application" << endl;
	cout << endl << endl;

	//--------------------------------------------------------------------------
	// OPENGL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

	// initialize GLFW library
	if (!glfwInit())
	{
		cout << "failed initialization" << endl;
		cSleepMs(1000);
		return 1;
	}

	// set error callback
	glfwSetErrorCallback(errorCallback);

	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w = 0.9 * mode->width;
	int h = 0.9 * mode->height;
	int x = 0.5 * (mode->width - w);
	int y = 0.5 * (mode->height - h);

	// Needed to handle GUI scale based on screen resolution.
	window_scale = 3.0f * (double)((double)mode->width / 3840.0f + (double)mode->height / 2160.0f) / 2;

	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// set active stereo mode
	if (stereoMode == C_STEREO_ACTIVE)
	{
		glfwWindowHint(GLFW_STEREO, GL_TRUE);
	}
	else
	{
		glfwWindowHint(GLFW_STEREO, GL_FALSE);
	}

	// create display context
	window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
	if (!window)
	{
		cout << "failed to create window" << endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}

	// get width and height of window
	glfwGetWindowSize(window, &width, &height);

	// set position of window
	glfwSetWindowPos(window, x, y);

	// set key callback
	glfwSetKeyCallback(window, keyCallback);

	// set mouse scroll callback
	glfwSetScrollCallback(window, scrollCallback);

	// set resize callback
	glfwSetWindowSizeCallback(window, windowSizeCallback);

	// set current display context
	glfwMakeContextCurrent(window);

	// sets the swap interval for the current display context
	glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
	// initialize GLEW library
	if (glewInit() != GLEW_OK)
	{
		cout << "failed to initialize GLEW library" << endl;
		glfwTerminate();
		return 1;
	}
#endif


	//--------------------------------------------------------------------------
	/************ Setup Dear ImGui context ************/
	//--------------------------------------------------------------------------

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();

	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL2_Init();

	io.FontGlobalScale = window_scale;

	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	world->m_backgroundColor.setBlack();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);

	// pivot objects for camera rotation
	center = new cGenericObject();
	center0 = new cGenericObject();

	world->addChild(center);
	world->addChild(center0);

	// rotating the center object will also rotate the camera
	center->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(-0.5, 0.0, 0.1),    // camera position (eye)
		cVector3d(0.0, 0.0, 0.0),    // look at position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

	cameraZeroTransform = camera->getLocalTransform();

	// set the near and far clipping planes of the camera
	camera->setClippingPlanes(0.01, 10.0);

	// set stereo mode
	camera->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoEyeSeparation(0.01);
	camera->setStereoFocalLength(0.5);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	// create a directional light source
	light1 = new cDirectionalLight(world);
	light2 = new cDirectionalLight(world);

	// insert light source inside world
	world->addChild(light1);
	world->addChild(light2);

	// enable light source
	light1->setEnabled(true);
	light2->setEnabled(true);

	// define direction of light beam
	light1->setDir(0.0, 0.0, -1.0);
	light1->setLocalPos(0.0, 0.0, 3.0);

	light2->setDir(0.0, 0.0, 1.0);
	light2->setLocalPos(0.0, 0.0, -3.0);

	// create a level to display Error data
	levelError = new cLevel();
	//camera->m_frontLayer->addChild(levelError);
	levelError->setLocalPos(20, 60);
	levelError->setRange(0.0, maxerror);
	levelError->setWidth(80);
	levelError->setNumIncrements(100);
	levelError->setSingleIncrementDisplay(false);
	levelError->setTransparencyLevel(0.5);

	/*****HandModel initilization*****/
	HandModel = new cHand();
	world->addChild(HandModel);

	//vector<int> dofvec{ 5,4,4,4,4 };
	vector<int> dofvec{ 5,4,5,5,5 }; // with arcpalm

	std::vector<std::vector<cTransform>> newT = HandModel->t_default_Tkach25Dof;

	// Tkach model with modified fingers lenghts
	for (int fingerid = 0; fingerid < newT.size(); fingerid++)
	{
		for (int fingerndofcounter = 0; fingerndofcounter < newT[fingerid].size(); fingerndofcounter++)
		{
			//cout << "Before: ";
			//printcTransform(newT[fingerid][fingerndofcounter],"T");

			newT[fingerid][fingerndofcounter].set(
				fingertransl[fingerid][fingerndofcounter],
				newT[fingerid][fingerndofcounter].getLocalRot()
			);

			//cout << "After: ";
			//printcTransform(newT[fingerid][fingerndofcounter],"T");
		}
	}


	HandModel->initialize(newT);


	vec = vecstart;

	vecbuffer = vec;

	for (int i = 0; i < HandModel->getdof(); i++)
	{
		inputname.push_back("Joint " + std::to_string(i + 1));
	}

	double cylr = 0.0065;

	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font
	font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	camera->m_frontLayer->addChild(labelRates);

	// setup callback when application exits
	atexit(close);

	//--------------------------------------------------------------------------
	// MAIN GRAPHIC LOOP
	//--------------------------------------------------------------------------

	// call window size callback at initialization
	windowSizeCallback(window, width, height);

	// main graphic loop
	while (!glfwWindowShouldClose(window))
	{
		// get width and height of window
		glfwGetWindowSize(window, &width, &height);


		//***************************** Change background color
		glClearColor(0.7, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT);

		// render graphics
		updateGraphics(); // update for Hand kinematics is in here!

		// swap buffers
		glfwSwapBuffers(window);

		// process events
		glfwPollEvents();

		// signal frequency counter
		freqCounterGraphics.signal(1);
	}


	// Cleanup
	ImGui_ImplOpenGL2_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	// close window
	glfwDestroyWindow(window);

	// terminate GLFW library
	glfwTerminate();

	// exit
	return 0;
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
	// update window size
	width = a_width;
	height = a_height;

}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
	cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
	// zoom in/out
	cVector3d posBuffer = camera->getLocalPos();

	posBuffer.normalize();

	camera->setLocalPos(camera->getLocalPos() - 0.1 * ((yoffset >= 0) - (yoffset <= 0)) * posBuffer);
	if (camera->getLocalPos().length() < 0.1)
	{
		camera->setLocalPos(0.1 * posBuffer);
	}
}
//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
	// filter calls that only include a key press
	if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
	{
		return;
	}

	// option - exit
	else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
	{
		glfwSetWindowShouldClose(a_window, GLFW_TRUE);
	}

	// toggle axes arrows
	else if ((a_key == GLFW_KEY_V) || (a_key == GLFW_KEY_Q))
	{
		HandModel->toggleArrows();
	}

	// option - reset view
	else if (a_key == GLFW_KEY_R)
	{
		cMatrix3d I;
		I.identity();
		center->setLocalRot(I);
		camera->setLocalTransform(cameraZeroTransform);
	}

	// option - rotate up
	else if (a_key == GLFW_KEY_W)
	{
		cMatrix3d Rold(center->getLocalRot());

		cVector3d axrot_in_local(cVector3d(0, 1, 0));
		Rold.mul(axrot_in_local);


		cMatrix3d R(axrot_in_local, -0.05);
		R *= Rold;

		center->setLocalRot(R);
	}

	// option - rotate down
	else if (a_key == GLFW_KEY_S)
	{
		cMatrix3d Rold(center->getLocalRot());

		cVector3d axrot_in_local(cVector3d(0, 1, 0));
		Rold.mul(axrot_in_local);


		cMatrix3d R(axrot_in_local, 0.05);
		R *= Rold;

		center->setLocalRot(R);
	}

	// option - rotate left
	else if (a_key == GLFW_KEY_A)
	{
		cMatrix3d Rold(center->getLocalRot());

		cVector3d axrot_in_local(cVector3d(0, 0, 1));
		Rold.mul(axrot_in_local);


		cMatrix3d R(axrot_in_local, 0.05);
		R *= Rold;

		center->setLocalRot(R);
	}

	// option - rotate right
	else if (a_key == GLFW_KEY_D)
	{
		cMatrix3d Rold(center->getLocalRot());

		cVector3d axrot_in_local(cVector3d(0, 0, 1));
		Rold.mul(axrot_in_local);


		cMatrix3d R(axrot_in_local, -0.05);
		R *= Rold;

		center->setLocalRot(R);
	}

	// option - show reference frame
	else if (a_key == GLFW_KEY_F)
	{
		// toggle state variable
		center0->setShowFrame(!center0->getShowFrame());
	}

	// option - toggle vertical mirroring
	else if (a_key == GLFW_KEY_M)
	{
		mirroredDisplay = !mirroredDisplay;
		camera->setMirrorVertical(mirroredDisplay);
	}
}

//------------------------------------------------------------------------------

void close(void)
{
	delete world;
}

//------------------------------------------------------------------------------

static void HelpMarker(const char* desc)
{
	ImGui::TextDisabled("(?)");
	if (ImGui::IsItemHovered())
	{
		ImGui::BeginTooltip();
		ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
		ImGui::TextUnformatted(desc);
		ImGui::PopTextWrapPos();
		ImGui::EndTooltip();
	}
}

//------------------------------------------------------------------------------
void updateGraphics(void)
{
	if (playdata) {
		posecount++;
		if (posecount < 0) {
			posecount = 0;
		}
		if (posecount > totalsteps) {
			posecount = totalsteps;
		}
	}

	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	// update haptic and graphic rate data
	labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / ");

	// update position of label
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world
	camera->renderView(width, height);

	// update hand model kinematics

	if (totalsteps > 0) {
		vec = jointangles_data_mat[posecount];

		for (int i = 0; i < HandModel->getdof(); i++) {
			vec[i] += vecstart[i]; // to be able to use an offset
		}
	}


	HandModel->updateAngles(vec);
	HandModel->updateKinematics();

	world->computeGlobalPositions(true);


	/*** Dear ImGui ***/
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL2_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	// render your GUI
	ImGui::Begin(" ");
	//ImGui::SetWindowSize(ImVec2(1090, 340));
	///ImVec2 imv = ImGui::GetWindowSize();

	/*for (int i = 0; i < HandModel->getdof(); i++) {
		//ImGui::InputDouble(inputname[i].c_str(), &vec[i], 0.01f, 1.0f, "%.8f");
		ImGui::DragScalar(inputname[i].c_str(), ImGuiDataType_Double, &vec[i], 0.01f);
	}*/

	//bool show_demo_window = true;
	//ImGui::ShowDemoWindow(&show_demo_window);

	ImGui::DragScalar("Step", ImGuiDataType_U16, &posecount, 2.0f);

	if (ImGui::Button("  Load Data  "))
	{
		// call function to load data from file
		jointangles_data_mat = loadDatFile(foldername + filename);

		totalsteps = jointangles_data_mat.size();
		dataloaded = TRUE;
		posecount = 0;

		vec = jointangles_data_mat[posecount];

		for (std::vector<double>::const_iterator i = vec.begin(); i != vec.end(); ++i)
			std::cout << *i << ' ';

		for (int i = 0; i < HandModel->getdof(); i++) {
			vec[i] += vecstart[i]; // to be able to use an offset
		}

	}

	ImGui::SameLine();

	//ImGui::SameLine();
	if (ImGui::Button(" Clear Data "))
	{
		// call function to clear data structure
		jointangles_data_mat.clear();

		dataloaded = FALSE;
		posecount = 0;
		totalsteps = 0;
		vec = vecstart;
	}

	if (ImGui::Button("  Play  "))
	{
		if (dataloaded) {
			playdata = TRUE;
		}
	}

	ImGui::SameLine();
	if (ImGui::Button("  Pause  "))
	{
		playdata = FALSE;
	}

	ImGui::SameLine();
	if (ImGui::Button(" Stop  "))
	{
		playdata = FALSE;
		posecount = 0;

		vec = jointangles_data_mat[posecount];

		for (int i = 0; i < HandModel->getdof(); i++) {
			vec[i] += vecstart[i]; // to be able to use an offset
		}
	}

	ImGui::SameLine();
	if (ImGui::Button(" Forward "))
	{
		if (dataloaded & !playdata) {
			posecount++;
			if (posecount < 0) {
				posecount = 0;
			}
			if (posecount > jointangles_data_mat.size()) {
				posecount = jointangles_data_mat.size();
			}

			vec = jointangles_data_mat[posecount];

			for (int i = 0; i < HandModel->getdof(); i++) {
				vec[i] += vecstart[i]; // to be able to use an offset
			}
		}
	}

	ImGui::SameLine();
	if (ImGui::Button(" Back  "))
	{
		if (dataloaded) {
			posecount--;
			if (posecount < 0) {
				posecount = 0;
			}
			if (posecount > jointangles_data_mat.size()) {
				posecount = jointangles_data_mat.size();
			}

			vec = jointangles_data_mat[posecount];

			for (int i = 0; i < HandModel->getdof(); i++) {
				vec[i] += vecstart[i]; // to be able to use an offset
			}
		}
	}

	ImGui::TextWrapped("Use W/S and A/D to rotate, R to reset view, V to show joint axes and F to show base reference frame.");

	ImGui::End();

	// Rendering
	ImGui::Render();

	ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

	

	// wait until all OpenGL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//
double rmse_from_vec(std::vector<double> estimate, std::vector<double> baseline)
{
	// calculate error vector
	std::vector<double> v_error;

	std::set_difference(estimate.begin(), estimate.end(),
		baseline.begin(), baseline.end(),
		std::inserter(v_error, v_error.begin())
	);

	// calculate rmse
	double mse = 0;
	for (unsigned int i = 0; i < v_error.size(); i++)
	{
		mse += v_error[i] * v_error[i];
	}
	mse = mse / v_error.size();
	return(sqrt(mse));
}
//------------------------------------------------------------------------------
vector<vector<double>> loadDatFile(string filename)
{
	std::vector<vector<double>> jointanglesmat;

	std::ifstream myfile;
	myfile.open(filename);

	std::string line;
	int count = 0;
	while (std::getline(myfile, line))
	{
		jointanglesmat.push_back(std::vector<double>());

		std::istringstream iss(line);

		std::copy(std::istream_iterator<float>(iss),
			std::istream_iterator<float>(),
			std::back_inserter(jointanglesmat[count]));

		count++;
	}

	return(jointanglesmat);
}

void getCameraParameters()
{
	cQuaternion cQuat;
	cQuat.fromRotMat(center->getLocalRot()); // rotation as quaternion

	cVector3d cameraLocalPos = camera->getLocalPos(); // position
}

void printcTransform(cTransform T, string Tname)
{
	std::cout << Tname << ": " << std::endl;
	cTransform dummy = T;
	std::cout << dummy(0, 0) << " " << dummy(0, 1) << " " << dummy(0, 2) << " " << dummy(0, 3) << " " << std::endl;
	std::cout << dummy(1, 0) << " " << dummy(1, 1) << " " << dummy(1, 2) << " " << dummy(1, 3) << " " << std::endl;
	std::cout << dummy(2, 0) << " " << dummy(2, 1) << " " << dummy(2, 2) << " " << dummy(2, 3) << " " << std::endl;
	std::cout << dummy(3, 0) << " " << dummy(3, 1) << " " << dummy(3, 2) << " " << dummy(3, 3) << " " << std::endl;
}
