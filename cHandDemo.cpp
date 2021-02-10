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

#include <string>
#include <iostream>
#include <cstdio>
#include <windows.h>



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
//Serial Communication Settings
HANDLE hSerial;                          // Handle to the Serial port
char  ComPortName[] = "\\\\.\\COM3";  // Name of the Serial port(May Change) to be opened,
BOOL  Status;                          // Status of the various operations 
DWORD dwEventMask;                     // Event mask to trigger
char  SerialBuffer[120000];               // Buffer Containing Rxed Data
bool send_char = false;

// low and high readings from the sensor (resistance), initialized to coarse estimates to be refined in calibration
int sensor_low = 17000;
int sensor_high = 160000;

// Pi
double Pi = 3.14159;

// Initial hand pose
std::vector<double> vecstart{
    0 * 30 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), -0 * 10 * (Pi / 180), -0 * 10 * (Pi / 180),
    0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
    0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
    0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
    0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180)
};

// First principal component (used to drive the hand shape from one sensor)
std::vector<double> PC1{
    0.03445656, 0.191433, -0.0310968, -0.07660658, -0.11885343,
    0.00317037, 0.22895402, 0.00754185, 0.31359066, 0.21799484,
    -0.00417658, 0.2861582, 0.00334999, 0.35693029, 0.27684938,
    -0.01066, 0.28191222, 0.00776594, 0.35906561, 0.23993875,
    -0.03012474, 0.25982226, 0.00517352, 0.27148087, 0.2919739
};

// Hand geometry (phalanxes lenght)
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

// a sphere to interact with
cShapeSphere* interaction_object;
double sph_radius = 0.8*0.035;
cVector3d interactionObjPos = cVector3d(-0.01, 0.002, -(sph_radius+0.01));

// Note: we can not initialize the following because we don't necessarily know the hand size
std::vector < std::vector<BOOL>> contactStatus;
// So to prevent the haptic feedback from triggering on the first loop we initialize this to true
BOOL hapticDelivered = TRUE;

// a camera to render the world in the window display
cCamera* camera;
cTransform cameraZeroTransform;

// Light sources
cDirectionalLight* light1;
cDirectionalLight* light2;

/* Needed for cHand */
// a hand object
cHand* HandModel;

// vectors to store joint angles
std::vector<double> vec;
std::vector<double> vecbuffer;
std::vector<double> vecref;
float PCcoeff = 0;

// GUI
std::vector<string> inputname; // used by the GUI to name each input (joint angle)
static float window_scale = 1.0f; // global window scale
static bool run_check = false;
/***************************************************************/

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

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

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// callback for when the mouse is scrolled
void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

// Serial communication functions
int open_port();
int readSerial();
BOOL writeSerial(char);

// function to check contact with the object
std::vector <std::vector <BOOL>> checkSphereContact(double offset = 0.01);
float PCcoeffContact = 0;
//==============================================================================
/*
    DEMO:   cHandDemo.cpp

    This application shows an example of how to use the cHand library for hand
    pose visualization in chai3D.

    A Flexpoint 3" bydirectional bend sensor is read through an Arduino micro.
    This is used to control the level of opening of the visualized hand, which
    closes and open according to a synergy. Interaction with an object is detected
    and feedback is provided through a vibrating motor on the fingertip.

    We refer to the World Haptics 2021 paper associated with cHand for more details.
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
    cout << "Demo: cHandDemo" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Enable/Disable potential field" << endl;
    cout << "[2] - Enable/Disable damping" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    /*---------------------------------- Opening the Serial Port -------------------------------------------*/
    open_port();


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
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
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
    camera->set(cVector3d(-0.5, 0.0, 0.1),      // camera position (eye)
        cVector3d(0.0, 0.0, 0.0),               // look at position (target)
        cVector3d(0.0, 0.0, 1.0));              // direction of the (up) vector

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

    // create a sphere (cursor) to represent the haptic device
    interaction_object = new cShapeSphere(sph_radius);
    interaction_object->setLocalPos(interactionObjPos);
    interaction_object->m_material->setGreenLime();

    world->addChild(interaction_object);

    /*****HandModel initilization*****/
    HandModel = new cHand();
    world->addChild(HandModel);

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

    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

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
        //glClearColor(0.7, 0, 0, 1);
        //glClear(GL_COLOR_BUFFER_BIT);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // GUI Cleanup
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

    else if (a_key == GLFW_KEY_C)
    {
        send_char = true;
    }

    else if (a_key == GLFW_KEY_P)
    {
        contactStatus = checkSphereContact();
        for (int fingerid = 0; fingerid < HandModel->getnfingers(); fingerid++)
        {
           for (int fingerndofcounter = 0; fingerndofcounter < HandModel->getdof_finger(fingerid); fingerndofcounter++)
           {
               //cout << contactStatus[fingerid][fingerndofcounter] << endl;
           }
        }
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // delete resources
    delete hapticsThread;
    delete world;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

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


    vecbuffer = vec;
    vecref = vec;

    float localPCcoeff = PCcoeff;

    std::transform(PC1.begin(), PC1.end(), vecref.begin(),
        [localPCcoeff](float f) { return f * PCcoeff; });

    contactStatus = checkSphereContact();

    BOOL atleastone_contact = FALSE;

    int blockjointcount = 0; // needed for the general case of a hand with different d.o.f. in different fingers
    for (int fingerid = 0; fingerid < HandModel->getnfingers(); fingerid++)
    {
        if (fingerid > 0)
        {
            blockjointcount += HandModel->getdof_finger(fingerid - 1);
        }

        for (int fingerndofcounter = 1; fingerndofcounter <= HandModel->getdof_finger(fingerid); fingerndofcounter++)
        {
            if (contactStatus[fingerid][fingerndofcounter])
            {
                atleastone_contact = TRUE;

                // if there is any contact flag haptic feedback activation, if it has not been delivered already
                if (!hapticDelivered)
                {
                    send_char = true;
                }
                
                // all proximal joints can not increase

                for (int distal_jointID = 0; distal_jointID < fingerndofcounter; distal_jointID++)
                {
                    if (abs(vecref[distal_jointID + blockjointcount]) >
                        abs(vecbuffer[distal_jointID + blockjointcount]))
                    {
                        vecref[distal_jointID + blockjointcount] =
                            vecbuffer[distal_jointID + blockjointcount];
                    }
                }
            }

        }
    }

    if (!atleastone_contact) { 
        hapticDelivered = FALSE; 
        interaction_object->m_material->setGreenLime();
    }
    else { interaction_object->m_material->setRed(); }
    vec = vecref;


    HandModel->updateAngles(vec);
    HandModel->updateKinematics();

    world->computeGlobalPositions(true);


    /*** Dear ImGui ***/
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // render your GUI
    ImGui::Begin("GUI");

    ImGui::Checkbox("Simulation running", &run_check);

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

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    int arduino_read;

    // main haptic simulation loop
    while (simulationRunning)
    {
        if (run_check) { // only do this is the checkbox option is selected
            //Read Serial port
            arduino_read = readSerial();
            cout << arduino_read << endl;
            
            float arduino_read_scaled = ((float)(arduino_read - sensor_low) / (float)(sensor_high - sensor_low)) * (Pi / 2);

            // We want a global coefficient PCcoeff so that if vec = PCcoeff*PC1 we have vec[11] = arduino_read_scaled;
            // vec[11] corresponds to the metacarpophalangeal joint on the middle finger which is where the bend sensor
            // is placed.
            PCcoeff = arduino_read_scaled / PC1[11];
            
            //cout << "PCcoeff = " << PCcoeff << endl;

            if (send_char) {
                writeSerial('1');
                hapticDelivered = true;
                send_char = false;
            }

        }
    }

    // exit haptics thread
    CloseHandle(hSerial); //Closing the Serial Port
    simulationFinished = true;
}

//------------------------------------------------------------------------------

BOOL writeSerial(char c)
{
    cout << "start" << endl;
    DWORD written;

    Status = WriteFile(hSerial, &c, sizeof(c), &written, NULL);
    cout << c << endl;
    return Status;
}

//------------------------------------------------------------------------------

int readSerial()
{

    DWORD dwBytesRead = 0;

    char TempChar;
    char SerialBuffer[10000] = { 0 };

    int serial_read_count = 0;
    int arduino_read;

    do
    {
        Status = ReadFile(hSerial, &TempChar, sizeof(TempChar), &dwBytesRead, NULL);
        SerialBuffer[serial_read_count] = TempChar;
        //cout << TempChar << endl;
        if (TempChar == '\r')
        {
            arduino_read = atoi(SerialBuffer);
            //cout << arduino_read << '\n';
            break;
        }

        serial_read_count++;
    } while (1);

    return arduino_read;

}

//------------------------------------------------------------------------------


int open_port()
{

    //Tell user that port-opening has been initiated
    printf("Opening Serial Port...\n");

    //Announce current task
    printf("Attaching handle...");

    //Attempt to attach a hande to serial port
    hSerial = CreateFile("COM3",

        GENERIC_READ | GENERIC_WRITE,
        0,
        0,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        0);

    if (hSerial == INVALID_HANDLE_VALUE)
    {

        if (GetLastError() == ERROR_FILE_NOT_FOUND) {

            //Print Error if neccessary
            printf("ERROR: Handle was not attached.  Reason: COM3 not available.\n");
            return 0;

        }
        
        return 0;

    }
    else printf("done!");                                //Successfully Attached handle to serial port

    //Announce current task
    printf("\nSetting Parameters...");

    //Set serial port parameters

    DCB dcbSerialParams = { 0 };
    //dcbSerial.DCBlength=sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {

        printf("failed!");
        return 0;

    }
    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(hSerial, &dcbSerialParams)) {

        printf("\n\nALERT: Serial port failed!");

    }
    else
    {
        printf("\n\n    Setting DCB Structure Successfull\n");
        printf("\n       Baudrate = %d", dcbSerialParams.BaudRate);
        printf("\n       ByteSize = %d", dcbSerialParams.ByteSize);
        printf("\n       StopBits = %d", dcbSerialParams.StopBits);
        printf("\n       Parity   = %d", dcbSerialParams.Parity);
    }


    /*------------------------------------ Setting Timeouts --------------------------------------------------*/
    /* Necessary to avoid the code to hang while waiting for data if the device gets unplugged */
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50; // in milliseconds
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (SetCommTimeouts(hSerial, &timeouts) == FALSE)
        printf("\n\n    Error! in Setting Time Outs");
    else
        printf("\n\n    Setting Serial Port Timeouts Successfull");

    /*------------------------------------ Setting Receive Mask ----------------------------------------------*/

    Status = SetCommMask(hSerial, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception

    if (Status == FALSE)
        printf("\n\n    Error! in Setting CommMask");
    else
        printf("\n\n    Setting CommMask successfull");

    return 1;

}

//------------------------------------------------------------------------------

std::vector <std::vector <BOOL>> checkSphereContact(double offset)
{
    std::vector <std::vector <BOOL>> isContact;
    std::vector<std::vector<cVector3d*>> handCentersPos = HandModel->getHandCenters();

    for (int fingerid = 0; fingerid < HandModel->getnfingers(); fingerid++)
    {
        isContact.push_back(std::vector<BOOL>());
        for (int fingerndofcounter = 0; fingerndofcounter <= HandModel->getdof_finger(fingerid); fingerndofcounter++)
        {
            isContact[fingerid].push_back(FALSE);
            // check if it is inside the sphere
            cVector3d tmp_distance = *handCentersPos[fingerid][fingerndofcounter] - interaction_object->getGlobalPos();
            
            //cout << tmp_distance.length() << endl;
            
            if (tmp_distance.length() <= sph_radius + offset) {
                isContact[fingerid][fingerndofcounter] = TRUE;
                //cout << isContact << endl;
            }
        }
    }

    return(isContact);

}