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
    \author    Francois Conti
	\version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

// cHand and this file authored by Edoardo Battaglia
//------------------------------------------------------------------------------

#include "chai3d.h"
#include "cHand.h"
#include <vector>
//------------------------------------------------------------------------------
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl2.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
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


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// Pi
double Pi = 3.14159;
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
cDirectionalLight *light1;
cDirectionalLight *light2;

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

/**** cHand: manual initialization of transforms for kinematics ******/
// a vector of transforms to define a finger
cVector3d f1joint1pos(-0.05f, 0.045f, -0.0f);
cVector3d f1joint2pos(0.00f, 0.0f, 0.0f);
cVector3d f1joint3pos(0.045f, 0.0f, 0.0f);
cVector3d f1joint4pos(0.03f, 0.0f, 0.0f);

cMatrix3d f1joint1rot(cVector3d(1, 0, 0), 0);
cMatrix3d f1joint2rot(cVector3d(1, 0, 0), -90 * (Pi / 180));
cMatrix3d f1joint3rot(cVector3d(1, 0, 0), 90 * (Pi / 180));
cMatrix3d f1joint4rot(cVector3d(1, 0, 0), 0);

cVector3d f2joint1pos(0.0f, 0.025f, 0.0f);
cVector3d f2joint2pos(0.0f, 0.0f, 0.0f);
cVector3d f2joint3pos(0.045f, 0.0f, 0.0f);
cVector3d f2joint4pos(0.03f, 0.0f, 0.0f);

cMatrix3d f2joint1rot(cVector3d(1, 0, 0), 0);
cMatrix3d f2joint2rot(cVector3d(1, 0, 0), -90 * (Pi / 180));
cMatrix3d f2joint3rot(cVector3d(1, 0, 0), 0 * (Pi / 180));
cMatrix3d f2joint4rot(cVector3d(1, 0, 0), 0 * (Pi / 180));

/*cVector3d f3joint1pos(0.0f, 0.0f, 0.0f);
cVector3d f3joint2pos(0.0f, 0.0f, 0.0f);
cVector3d f3joint3pos(0.045f, 0.0f, 0.0f);
cVector3d f3joint4pos(0.03f, 0.0f, 0.0f);

cMatrix3d f3joint1rot(cVector3d(1, 0, 0), 0);
cMatrix3d f3joint2rot(cVector3d(1, 0, 0), -90 * (Pi / 180));
cMatrix3d f3joint3rot(cVector3d(1, 0, 0), 0 * (Pi / 180));
cMatrix3d f3joint4rot(cVector3d(1, 0, 0), 0 * (Pi / 180));*/

cVector3d f4joint1pos(0.0f, -0.025f, 0.0f);
cVector3d f4joint2pos(0.0f, 0.0f, 0.0f);
cVector3d f4joint3pos(0.04f, 0.0f, 0.0f);
cVector3d f4joint4pos(0.025f, 0.0f, 0.0f);

cMatrix3d f4joint1rot(cVector3d(1, 0, 0), 0);
cMatrix3d f4joint2rot(cVector3d(1, 0, 0), -90 * (Pi / 180));
cMatrix3d f4joint3rot(cVector3d(1, 0, 0), 0 * (Pi / 180));
cMatrix3d f4joint4rot(cVector3d(1, 0, 0), 0 * (Pi / 180));

/*cVector3d f5joint1pos(0.0f, -0.05f, 0.0f);
cVector3d f5joint2pos(0.0f, -0.0f, 0.0f);
cVector3d f5joint3pos(0.035f, 0.0f, 0.0f);
cVector3d f5joint4pos(0.02f, 0.0f, 0.0f);

cMatrix3d f5joint1rot(cVector3d(1, 0, 0), 0);
cMatrix3d f5joint2rot(cVector3d(1, 0, 0), -90 * (Pi / 180));
cMatrix3d f5joint3rot(cVector3d(1, 0, 0), 0 * (Pi / 180));
cMatrix3d f5joint4rot(cVector3d(1, 0, 0), 0 * (Pi / 180));*/

cTransform Tf1j1(f1joint1pos, f1joint1rot);
cTransform Tf1j2(f1joint2pos, f1joint2rot);
cTransform Tf1j3(f1joint3pos, f1joint3rot);
cTransform Tf1j4(f1joint4pos, f1joint4rot);

cTransform Tf2j1(f2joint1pos, f2joint1rot);
cTransform Tf2j2(f2joint2pos, f2joint2rot);
cTransform Tf2j3(f2joint3pos, f2joint3rot);
cTransform Tf2j4(f2joint4pos, f2joint4rot);

/*cTransform Tf3j1(f3joint1pos, f3joint1rot);
cTransform Tf3j2(f3joint2pos, f3joint2rot);
cTransform Tf3j3(f3joint3pos, f3joint3rot);
cTransform Tf3j4(f3joint4pos, f3joint4rot);*/

cTransform Tf4j1(f4joint1pos, f4joint1rot);
cTransform Tf4j2(f4joint2pos, f4joint2rot);
cTransform Tf4j3(f4joint3pos, f4joint3rot);
cTransform Tf4j4(f4joint4pos, f4joint4rot);

/*cTransform Tf5j1(f5joint1pos, f5joint1rot);
cTransform Tf5j2(f5joint2pos, f5joint2rot);
cTransform Tf5j3(f5joint3pos, f5joint3rot);
cTransform Tf5j4(f5joint4pos, f5joint4rot);*/

vector<cTransform> Tf1;
vector<cTransform> Tf2;
//vector<cTransform> Tf3;
vector<cTransform> Tf4;
//vector<cTransform> Tf5;

vector<vector<cTransform>> Tmodel;

/***************************************************************/
// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a handle to window display context
GLFWwindow* window = NULL;

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

/* Debug utilities */
void printcTransform(cTransform T, string Tname);

//==============================================================================
/*
	DEMO:   HandVisualizerGUI_TransformsDefinition_3fingers.cpp

	This application illustrates how to initialize and visualize a hand model
	using cHand in Chai3D.

	In this example the application opens an OpenGL window and displays a
	3D hand model with a simple GUI to change joint angles. The GUI is built using
	Dear ImGUI and all components needed for the GUI are included in this folder.

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

	// GUI scale is handled based on screen resolution.
	window_scale = 3.0f*(double)((double)mode->width / 3840.0f + (double)mode->height / 2160.0f) / 2;

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

	/*****HandModel initilization*****/
	Tf1.push_back(Tf1j1);
	Tf1.push_back(Tf1j2);
	Tf1.push_back(Tf1j3);
	Tf1.push_back(Tf1j4);

	Tf2.push_back(Tf2j1);
	Tf2.push_back(Tf2j2);
	Tf2.push_back(Tf2j3);
	Tf2.push_back(Tf2j4);

	/*Tf3.push_back(Tf3j1);
	Tf3.push_back(Tf3j2);
	Tf3.push_back(Tf3j3);
	Tf3.push_back(Tf3j4);*/

	Tf4.push_back(Tf4j1);
	Tf4.push_back(Tf4j2);
	Tf4.push_back(Tf4j3);
	Tf4.push_back(Tf4j4);

	/*Tf5.push_back(Tf5j1);
	Tf5.push_back(Tf5j2);
	Tf5.push_back(Tf5j3);
	Tf5.push_back(Tf5j4);*/

	Tmodel.push_back(Tf1);
	Tmodel.push_back(Tf2);
	//Tmodel.push_back(Tf3);
	Tmodel.push_back(Tf4);
	//Tmodel.push_back(Tf5);

	HandModel = new cHand();
	world->addChild(HandModel);
	
	/* DLR */
	bool DLR = false;
	
	HandModel->initialize(Tmodel);

	/* For 20 dof model*/
	std::vector<double> vecstart{
		10 * (Pi / 180), 20 * (Pi / 180), -10 * (Pi / 180), -10 * (Pi / 180),
		0 * (Pi / 180), 5 * (Pi / 180), 10 * (Pi / 180), 10 * (Pi / 180),
		//0 * (Pi / 180), 5 * (Pi / 180), 10 * (Pi / 180), 10 * (Pi / 180),
		0 * (Pi / 180), 5 * (Pi / 180), 10 * (Pi / 180), 10 * (Pi / 180)
		//0 * (Pi / 180), 5 * (Pi / 180), 10 * (Pi / 180), 10 * (Pi / 180)
	};

	vec = vecstart;

	vecbuffer = vec;

	for (int i = 0; i < HandModel->getdof(); i++)
	{
		inputname.push_back("Joint " + std::to_string(i + 1));
	}

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

	camera->setLocalPos(camera->getLocalPos() - 0.1*((yoffset > 0) - (yoffset < 0))*posBuffer);
	if (camera->getLocalPos().length() < 0.1)
	{
		camera->setLocalPos(0.1*posBuffer);
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
	HandModel->updateAngles(vec);
	HandModel->updateKinematics();

	// Important: without the next line rendering of the hand will not be updated
	world->computeGlobalPositions(true);

	/*** Dear ImGui ***/
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL2_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	// render your GUI
	ImGui::Begin("Joint angles");
	ImVec2 imv = ImGui::GetWindowSize();
	
	for (int i = 0; i < HandModel->getdof(); i++) {
		ImGui::DragScalar(inputname[i].c_str(), ImGuiDataType_Double, &vec[i], 0.01f);
	}

	if (ImGui::Button(" Zero pose "))
	{
		vec = std::vector<double>(HandModel->getdof(),0);
	}
	ImGui::SameLine();
	if (ImGui::Button(" Save pose "))
	{
		vecbuffer = vec;
	}
	ImGui::SameLine();
	if (ImGui::Button(" Restore pose "))
	{
		vec = vecbuffer;
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

//------------------------------------------------------------------------------

void printcTransform(cTransform T, string Tname)
{
	std::cout << Tname << ": " << std::endl;
	cTransform dummy = T;
	std::cout << dummy(0, 0) << " " << dummy(0, 1) << " " << dummy(0, 2) << " " << dummy(0, 3) << " " << std::endl;
	std::cout << dummy(1, 0) << " " << dummy(1, 1) << " " << dummy(1, 2) << " " << dummy(1, 3) << " " << std::endl;
	std::cout << dummy(2, 0) << " " << dummy(2, 1) << " " << dummy(2, 2) << " " << dummy(2, 3) << " " << std::endl;
	std::cout << dummy(3, 0) << " " << dummy(3, 1) << " " << dummy(3, 2) << " " << dummy(3, 3) << " " << std::endl;
}
