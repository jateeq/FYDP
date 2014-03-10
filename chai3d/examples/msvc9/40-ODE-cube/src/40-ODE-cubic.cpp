//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-2010 by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   2.1.0 $Rev: 316 $
 */
//===========================================================================

//---------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CODE.h"
//---------------------------------------------------------------------------

//-----------------Jake--------------//
#include "SerialClass.h"
#include <iomanip>
#include <sstream>
#include <conio.h>
#include <dos.h>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <windows.h>
//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W         = 512;
const int WINDOW_SIZE_H         = 512;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

cVector3d previous_pos(0,0,0);
cVector3d dpos(0,0,0);
cVector3d previous_object_pos(0,0,0);

//-----------------Jake--------------//
cGeneric3dofPointer* thumb;
cGeneric3dofPointer* index_finger;
cLabel* labels[3];
cGenericObject* rootLabels;
Serial* sp;
double previous_pos1, previous_pos2;
double overall_pos1, overall_pos2 = 0;
double holding_pos1, holding_pos2 = 0;
double pos1_1, pos1_2, pos1_3, pos1_4, pos1_5 = 0;
double pos2_1, pos2_2, pos2_3, pos2_4, pos2_5 = 2;
int pos_counter = 0;
cShapeLine* index_tool_line;
cShapeLine* thumb_tool_line;
cMesh* drill;
clock_t time_before = 0, time_now = 0;

//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a light source to illuminate the objects in the virtual scene
cLight *light;

// a little "chai3d" bitmap logo at the bottom of the screen
cBitmap* logo;

// width and height of the current window display
int displayW  = 0;
int displayH  = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a virtual tool representing the haptic device in the scene
cGeneric3dofPointer* tool;

// radius of the tool proxy
double proxyRadius;

// a pointer the ODE object grasped by the tool
cODEGenericBody* graspObject;

// grasp position is respect to object
cVector3d graspPos;

// is grasp currently active?
bool graspActive = false;

// a small line used to display a grasp
cShapeLine* graspLine;

// maximum stiffness to be used with virtual objects in scene
double stiffnessMax;

// status of the main simulation haptics loop
bool simulationRunning = false;

// ODE world
cODEWorld* ODEWorld;

// ODE object
cODEGenericBody* ODEBody0;
cODEGenericBody* ODEBody1;
cODEGenericBody* ODEBody2;

cODEGenericBody* ODEGPlane0;
cODEGenericBody* ODEGPlane1;
cODEGenericBody* ODEGPlane2;
cODEGenericBody* ODEGPlane3;
cODEGenericBody* ODEGPlane4;
cODEGenericBody* ODEGPlane5;

// root resource path
string resourceRoot;

// has exited haptics simulation thread
bool simulationFinished = false;

//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

// function called before exiting the application
void close(void);

// main graphics callback
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);

// creates a cube mesh
void createCube(cMesh* a_mesh, double a_size);


//===========================================================================
/*
    DEMO:    ODE_cubic.cpp

    This example illustrates the use of the ODE framework for simulating
    haptic interaction with dynamic bodies. In this scene we create 3
	cubic meshes that we individually attach to ODE bodies. Haptic interactions
	are computer by using the finger-proxy haptic model and forces are
	propagated to the ODE representation.
 */
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    printf ("\n");
    printf ("-----------------------------------\n");
    printf ("CHAI 3D\n");
    printf ("Demo: 40-ODE-cubic\n");
    printf ("Copyright 2003-2010\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Instructions:\n\n");
    printf ("- Use haptic device and user switch to manipulate cubes \n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[1] - Enable gravity\n");
    printf ("[2] - Disable gravity\n");
    printf ("[x] - Exit application\n");
    printf ("\n\n");

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);

	sp = new Serial("COM6");
	if (sp->IsConnected())
		printf("We're connected\n");

    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    // the color is defined by its (R,G,B) components.
    world->setBackgroundColor(0.0, 0.0, 0.0);

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and oriente the camera
    camera->set( cVector3d (3.0, 0.0, 0.3),    // camera position (eye)
        cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
        cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // create a light source and attach it to the camera
    light = new cLight(world);
    camera->addChild(light);                   // attach light to camera
    light->setEnabled(true);                   // enable light source
    light->setPos(cVector3d( 2.0, 0.5, 1.0));  // position the light source
    light->setDir(cVector3d(-2.0, 0.5, 1.0));  // define the direction of the light beam
    light->m_ambient.set(0.6, 0.6, 0.6);
    light->m_diffuse.set(0.8, 0.8, 0.8);
    light->m_specular.set(0.8, 0.8, 0.8);


    //-----------------------------------------------------------------------
    // 2D - WIDGETS
    //-----------------------------------------------------------------------

    // create a 2D bitmap logo
    logo = new cBitmap();

    // add logo to the front plane
    camera->m_front_2Dscene.addChild(logo);

    // load a "chai3d" bitmap image file
    bool fileload;
    fileload = logo->m_image.loadFromFile(RESOURCE_PATH("resources/images/chai3d.bmp"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = logo->m_image.loadFromFile("../../../bin/resources/images/chai3d.bmp");
        #endif
    }

    // position the logo at the bottom left of the screen (pixel coordinates)
    logo->setPos(10, 10, 0);

    // scale the logo along its horizontal and vertical axis
    logo->setZoomHV(0.25, 0.25);

    // here we replace all black pixels (0,0,0) of the logo bitmap
    // with transparent black pixels (0, 0, 0, 0). This allows us to make
    // the background of the logo look transparent.
    logo->m_image.replace(
        cColorb(0, 0, 0),      // original RGB color
        cColorb(0, 0, 0, 0)    // new RGBA color
    );

    // enable transparency
    logo->enableTransparency(true);


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    cGenericHapticDevice* hapticDevice;
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo info;
    if (hapticDevice)
    {
        info = hapticDevice->getSpecifications();
    }

    // create a 3D tool and add it to the world
    tool = new cGeneric3dofPointer(world);

	//-----------------Jake--------------//
	index_finger = new cGeneric3dofPointer(world);
	thumb = new cGeneric3dofPointer(world);

    world->addChild(tool);
	world->addChild(index_finger);
	world->addChild(thumb);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // initialize tool by connecting to haptic device
    tool->start();

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);
	index_finger->setWorkspaceRadius(1.0);
	thumb->setWorkspaceRadius(1.0);

    // define a radius for the tool (graphical display)
    tool->setRadius(0.03);
	index_finger->setRadius(0.03);
	thumb->setRadius(0.03);

    // hide the device sphere. only show proxy.
    tool->m_deviceSphere->setShowEnabled(false);
	index_finger->setShowEnabled(false);
	thumb->setShowEnabled(false);

    // set the physical readius of the proxy.
    proxyRadius = 0.03;
    tool->m_proxyPointForceModel->setProxyRadius(proxyRadius);
	index_finger->m_proxyPointForceModel->setProxyRadius(proxyRadius);
	thumb->m_proxyPointForceModel->setProxyRadius(proxyRadius);

    tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;
	index_finger->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;
	thumb->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->m_proxyPointForceModel->m_useDynamicProxy = true;
	index_finger->m_proxyPointForceModel->m_useDynamicProxy = true;
	thumb->m_proxyPointForceModel->m_useDynamicProxy = true;

    // ajust the color of the tool
    tool->m_materialProxy = tool->m_materialProxyButtonPressed;
	index_finger->m_materialProxy = index_finger->m_materialProxyButtonPressed;
	thumb->m_materialProxy = thumb->m_materialProxyButtonPressed;

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // define a maximum stiffness that can be handled by the current
    // haptic device. The value is scaled to take into account the
    // workspace scale factor
    stiffnessMax = info.m_maxForceStiffness / workspaceScaleFactor;

    // create a small white line that will be enabled every time the operator
    // grasps an object. The line indicated the connection between the
    // position of the tool and the grasp position on the object
    graspLine = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
    world->addChild(graspLine);
    graspLine->m_ColorPointA.set(1.0, 1.0, 1.0);
    graspLine->m_ColorPointB.set(1.0, 1.0, 1.0);
    graspLine->setShowEnabled(false);

    index_tool_line = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
    world->addChild(index_tool_line);
    index_tool_line->m_ColorPointA.set(0.7, 0.7, 0.7);
    index_tool_line->m_ColorPointB.set(0.7, 0.7, 0.7);
    //index_tool_line->setShowEnabled(false);

    thumb_tool_line = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
    world->addChild(thumb_tool_line);
    thumb_tool_line->m_ColorPointA.set(0.5, 0.5, 0.5);
    thumb_tool_line->m_ColorPointB.set(0.5, 0.5, 0.5);
    //thumb_tool_line->setShowEnabled(false);

    // create a new mesh.
    drill = new cMesh(world);

    // load a drill like mesh and attach it to the tool
    fileload = drill->loadFromFile(RESOURCE_PATH("resources/models/drill/drill.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = drill->loadFromFile("../../../bin/resources/models/drill/drill.3ds");
        #endif
    }
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
        close();
        return (-1);
    }

    // resize tool mesh model
    drill->scale(0.004);

    // remove the collision detector. we do not want to compute any
    // force feedback rendering on the object itself.
    drill->deleteCollisionDetector(true);

    // define a material property for the mesh
    cMaterial mat;
    mat.m_ambient.set(0.5, 0.5, 0.5);
    mat.m_diffuse.set(0.8, 0.8, 0.8);
    mat.m_specular.set(1.0, 1.0, 1.0);
    drill->setMaterial(mat, true);
    drill->computeAllNormals(true);
	drill->rotate(cVector3d(0,1,0), -3.14/2);

    // attach drill to tool
    tool->m_proxyMesh->addChild(drill);
	tool->rotate(cVector3d(0,0,1), 3.14/2);

    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);

    // add ODE world as a node inside world
    world->addChild(ODEWorld);

    // set some gravity
    ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));

    // create a new ODE object that is automatically added to the ODE world
    ODEBody0 = new cODEGenericBody(ODEWorld);
    ODEBody1 = new cODEGenericBody(ODEWorld);
    ODEBody2 = new cODEGenericBody(ODEWorld);

    // create a virtual mesh  that will be used for the geometry
    // representation of the dynamic body
    cMesh* object0 = new cMesh(world);
    cMesh* object1 = new cMesh(world);
    cMesh* object2 = new cMesh(world);

    // crate a cube mesh
    double boxSize = 0.30;
    createCube(object0, boxSize);
    createCube(object1, boxSize);
    createCube(object2, boxSize);

    // define some material properties for each cube
    cMaterial mat0, mat1, mat2;
    mat0.m_ambient.set(0.8, 0.1, 0.4);
    mat0.m_diffuse.set(1.0, 0.15, 0.5);
    mat0.m_specular.set(1.0, 0.2, 0.8);
    mat0.setStiffness(0.5 * stiffnessMax);
    mat0.setDynamicFriction(0.8);
    mat0.setStaticFriction(0.8);
    object0->setMaterial(mat0);

    mat1.m_ambient.set(0.2, 0.6, 0.0);
    mat1.m_diffuse.set(0.2, 0.8, 0.0);
    mat1.m_specular.set(0.2, 1.0, 0.0);
    mat1.setStiffness(0.5 * stiffnessMax);
    mat1.setDynamicFriction(0.8);
    mat1.setStaticFriction(0.8);
    object1->setMaterial(mat1);

    mat2.m_ambient.set(0.0, 0.2, 0.6);
    mat2.m_diffuse.set(0.0, 0.2, 0.8);
    mat2.m_specular.set(0.0, 0.2, 1.0);
    mat2.setStiffness(0.5 * stiffnessMax);
    mat2.setDynamicFriction(0.8);
    mat2.setStaticFriction(0.8);
    object2->setMaterial(mat2);

    // add mesh to ODE object
    ODEBody0->setImageModel(object0);
    ODEBody1->setImageModel(object1);
    ODEBody2->setImageModel(object2);

    // create a dynamic model of the ODE object. Here we decide to use a box just like
    // the object mesh we just defined
    ODEBody0->createDynamicBox(boxSize, boxSize, boxSize);
    ODEBody1->createDynamicBox(boxSize, boxSize, boxSize, false, cVector3d(1,1,1));
    ODEBody2->createDynamicBox(boxSize, boxSize, boxSize);

    // define some mass properties for each cube
    ODEBody0->setMass(0.15);
    ODEBody1->setMass(0.30);
    ODEBody2->setMass(0.45);

    // set position of each cube
    cVector3d tmpvct;
    tmpvct = cVector3d(0.0,-0.6, -0.5);
    ODEBody0->setPosition(tmpvct);
    tmpvct = cVector3d(0.0, 0.6, -0.5);
    ODEBody1->setPosition(tmpvct);
    tmpvct = cVector3d(0.0, 0.0, -0.5);
    ODEBody2->setPosition(tmpvct);

    // rotate central cube of 45 degrees (just to show hoe this works!)
    cMatrix3d rot;
    rot.identity();
    rot.rotate(cVector3d(0,0,1), cDegToRad(45));
    ODEBody0->setRotation(rot);

    // we create 6 static walls to contains the 3 cubes within a limited workspace
    ODEGPlane0 = new cODEGenericBody(ODEWorld);
    ODEGPlane1 = new cODEGenericBody(ODEWorld);
    ODEGPlane2 = new cODEGenericBody(ODEWorld);
    ODEGPlane3 = new cODEGenericBody(ODEWorld);
    ODEGPlane4 = new cODEGenericBody(ODEWorld);
    ODEGPlane5 = new cODEGenericBody(ODEWorld);

    double size = 1.0;
    ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0,  2.0 *size), cVector3d(0.0, 0.0 ,-1.0));
    ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -size), cVector3d(0.0, 0.0 , 1.0));
    ODEGPlane2->createStaticPlane(cVector3d(0.0,  size, 0.0), cVector3d(0.0,-1.0, 0.0));
    ODEGPlane3->createStaticPlane(cVector3d(0.0, -size, 0.0), cVector3d(0.0, 1.0, 0.0));
    ODEGPlane4->createStaticPlane(cVector3d( size, 0.0, 0.0), cVector3d(-1.0,0.0, 0.0));
    ODEGPlane5->createStaticPlane(cVector3d(-0.8 * size, 0.0, 0.0), cVector3d( 1.0,0.0, 0.0));

    //////////////////////////////////////////////////////////////////////////
    // Create some reflexion
    //////////////////////////////////////////////////////////////////////////

    // we create an intermediate node to which we will attach
    // a copy of the object located inside the world
    cGenericObject* reflexion = new cGenericObject();

    // set this object as a ghost node so that no haptic interactions or
    // collision detecting take place within the child nodes added to the
    // reflexion node.
    reflexion->setAsGhost(true);

    // add reflexion node to world
    world->addChild(reflexion);

    // turn off culling on each object (objects now get rendered on both sides)
    object0->setUseCulling(false, true);
    object1->setUseCulling(false, true);
    object2->setUseCulling(false, true);

    // create a symmetry rotation matrix (z-plane)
    cMatrix3d rotRefexion;
    rotRefexion.set(1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, -1.0);
    reflexion->setRot(rotRefexion);
    reflexion->setPos(0.0, 0.0, -2.005);

    // add objects to the world
    reflexion->addChild(ODEWorld);
    reflexion->addChild(tool);

    //////////////////////////////////////////////////////////////////////////
    // Create a Ground
    //////////////////////////////////////////////////////////////////////////

    // create mesh to model ground surface
    cMesh* ground = new cMesh(world);
    world->addChild(ground);

    // create 4 vertices (one at each corner)
    double groundSize = 2.0;
    int vertices0 = ground->newVertex(-groundSize, -groundSize, 0.0);
    int vertices1 = ground->newVertex( groundSize, -groundSize, 0.0);
    int vertices2 = ground->newVertex( groundSize,  groundSize, 0.0);
    int vertices3 = ground->newVertex(-groundSize,  groundSize, 0.0);

    // compose surface with 2 triangles
    ground->newTriangle(vertices0, vertices1, vertices2);
    ground->newTriangle(vertices0, vertices2, vertices3);

    // compute surface normals
    ground->computeAllNormals();

    // position ground at the right level
    ground->setPos(0.0, 0.0, -1.0);

    // define some material properties and apply to mesh
    cMaterial matGround;
    matGround.setStiffness(stiffnessMax);
    matGround.setDynamicFriction(0.7);
    matGround.setStaticFriction(1.0);
    matGround.m_ambient.set(0.0, 0.0, 0.0);
    matGround.m_diffuse.set(0.0, 0.0, 0.0);
    matGround.m_specular.set(0.0, 0.0, 0.0);
    ground->setMaterial(matGround);

    // enable and set transparency level of ground
    ground->setTransparencyLevel(0.7);
    ground->setUseTransparency(true);

	//----Jake-----//

	string strID;
    cStr(strID, 0);
    string strDevice = "#" + strID + " - " +info.m_modelName;

	cLabel* newLabel = new cLabel();
	tool->m_proxyMesh->addChild(newLabel);
	newLabel->m_string = strDevice;
	newLabel->setPos(0.00, 0.05, 0.00);
	newLabel->m_fontColor.set(1.0, 1.0, 1.0);

	string strID1;
    cStr(strID1, 1);
    string strDevice1 = "#" + strID1 + " - index finger";

	cLabel* newLabel1 = new cLabel();
	index_finger->m_proxyMesh->addChild(newLabel1);
	newLabel1->m_string = strDevice1;
	newLabel1->setPos(0.00, 0.05, 0.00);
	newLabel1->m_fontColor.set(1.0, 1.0, 1.0);

	string strID2;
    cStr(strID2, 2);
    string strDevice2 = "#" + strID2 + " - thumb";

	cLabel* newLabel2 = new cLabel();
	thumb->m_proxyMesh->addChild(newLabel2);
	newLabel2->m_string = strDevice2;
	newLabel2->setPos(0.00, 0.05, 0.00);
	newLabel2->m_fontColor.set(1.0, 1.0, 1.0);

    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve the resolution of the computer display and estimate the position
    // of the GLUT window so that it is located at the center of the screen
    int screenW = glutGet(GLUT_SCREEN_WIDTH);
    int screenH = glutGet(GLUT_SCREEN_HEIGHT);
    int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
    int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI 3D");

    // create a mouse menu (right button)
    glutCreateMenu(menuSelect);
    glutAddMenuEntry("full screen", OPTION_FULLSCREEN);
    glutAddMenuEntry("window display", OPTION_WINDOWDISPLAY);
    glutAttachMenu(GLUT_RIGHT_BUTTON);


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // simulation in now running
    simulationRunning = true;

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

    // start the main graphics rendering loop
    glutMainLoop();

    // close everything
    close();

    // exit
    return (0);
    }

//---------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    // update the size of the viewport
    displayW = w;
    displayH = h;
    glViewport(0, 0, displayW, displayH);
}

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // escape key
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        close();

        // exit application
        exit(0);
    }

    // option 1:
    if (key == '1')
    {
        // enable gravity
        ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));
    }

    // option 2:
    if (key == '2')
    {
        // disable gravity
        ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
    }
}

//---------------------------------------------------------------------------

void menuSelect(int value)
{
    switch (value)
    {
        // enable full screen display
        case OPTION_FULLSCREEN:
        glutFullScreen();
        break;

        // reshape window to original size
        case OPTION_WINDOWDISPLAY:
        glutReshapeWindow(WINDOW_SIZE_W, WINDOW_SIZE_H);
        break;
    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();
    }

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    // render world
    camera->renderView(displayW, displayH);

    // Swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

    // inform the GLUT window to call updateGraphics again (next frame)
    if (simulationRunning)
    {
        glutPostRedisplay();
    }
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation clock
    cPrecisionClock simClock;
    simClock.start(true);
	int i = 0;
    // main haptic simulation loop
	double temp_x;
    while(simulationRunning)
    {
        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updatePose();

        // compute interaction forces
        tool->computeInteractionForces();

        // check if the tool is touching an object
		cGenericObject* object_tool = tool->m_proxyPointForceModel->m_contactPoint0->m_object;
        cGenericObject* object_index_finger = index_finger->m_proxyPointForceModel->m_contactPoint0->m_object;
		cGenericObject* object_thumb = thumb->m_proxyPointForceModel->m_contactPoint0->m_object;
		
        // read user switch status
        bool userSwitch;

        cVector3d pos = tool->m_proxyPointForceModel->getProxyGlobalPosition();
        cMatrix3d rot = tool->m_deviceGlobalRot;

		//-----------------Jake--------------//

		char serialData[50];
		double pos1 = 0;
		double pos2 = 0;
		time_now = GetTickCount();
		if (i == 0)
		{
			time_before = time_now;
			i++;
		}
		int seconds = (time_now-time_before);
		double dt = seconds/1000.0;
					//	printf("Elapsed time: %f seconds\n", dt);
		if ( dt > 0.100)
		{
			if (sp->ReadData(serialData, strlen(serialData)) > 0)
			{
				if (sp->parse_num(serialData, pos1, pos2))
				{
					if (pos1 > 0.3)
					{
 						pos1 = 0.126*pow(pos1,-1.07);
						pos1 = (pos1+pos1_1+pos1_2)/3;
						pos1_5 = pos1_4;
						pos1_4 = pos1_3;
						pos1_3 = pos1_2;
						pos1_2 = pos1_1;
						pos1_1 = pos1;

					} 
					else 
					{
						pos1 = previous_pos1;
					}

					if (pos2 > 0.3)
					{
 						pos2 = 0.126*pow(pos2,-1.07);
						pos2 = (pos2 + pos2_1 + pos2_2)/3;
						pos2_5 = pos2_4;
						pos2_4 = pos2_3;
						pos2_3 = pos2_2;
						pos2_2 = pos2_1;
						pos2_1 = pos2;
					} else 
					{
						pos2 = previous_pos2;
					}

					time_before = time_now;
					printf("Received data %f and %f \n", pos1, pos2);
				} else 
				{
					printf("Conversion failed\n");
					pos1 = previous_pos1;
					pos2 = previous_pos2;
				}
			
			} else {
				printf("Receive failed\n");
				pos1 = previous_pos1;
				pos2 = previous_pos2;
			}
		
		}else{
			pos1 = previous_pos1;
			pos2 = previous_pos2;
		}

		if (pos_counter > 2)
		{
			double dx1 = pos1 - previous_pos1;
			double dx2 = pos2 - previous_pos2;

			if (abs(dx1) < 0.1)
			{
				overall_pos1 += dx1*10;
				if (overall_pos1 > 0.1)
				{
					overall_pos1 = 0.1;
				} else if(overall_pos1 < 0)
				{
					overall_pos1 = 0;
				}
			}

			if (abs(dx2) < 0.1)
			{
				overall_pos2 -= dx2*10;
				if (overall_pos2 < -0.1)
				{
					overall_pos2 = -0.1;
				} else if(overall_pos2 > 0)
				{
					overall_pos2 = 0;
				}
			}
		} else {
			pos_counter++;
		}

		previous_pos1 = pos1;
		previous_pos2 = pos2;

		index_finger->setPos(pos.x, pos.y + 0.25 - overall_pos1, pos.z - 0.25);
		index_finger->setRot(rot);

		index_finger->computeInteractionForces();

		index_finger->updatePose();

		thumb->setPos(pos.x, pos.y - 0.25 - overall_pos2, pos.z - 0.25);
		thumb->setRot(rot);

		thumb->computeInteractionForces();

		thumb->updatePose();

		tool->m_lastComputedGlobalForce += index_finger->m_lastComputedGlobalForce + thumb->m_lastComputedGlobalForce;

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

		double torque_temp1 = sqrt(pow(index_finger->m_lastComputedGlobalForce.x,2) + pow(index_finger->m_lastComputedGlobalForce.y,2) + pow(index_finger->m_lastComputedGlobalForce.z,2));
		double torque_temp2 = sqrt(pow(thumb->m_lastComputedGlobalForce.x,2) + pow(thumb->m_lastComputedGlobalForce.y,2) + pow(thumb->m_lastComputedGlobalForce.z,2));

		char to_send[50];
		if (graspActive)
		{
			strcpy(to_send, "T");
			strcat(to_send, "1/1");
			strcat(to_send, ";");

			if (abs(overall_pos1 - holding_pos1) > 0.05|| abs(overall_pos2 - holding_pos2) > 0.05)
			{
				userSwitch = false;
			} else 
			{
				userSwitch = true;
			}
		} else
		{
			if (torque_temp1 > 0 && torque_temp2 > 0)
			{
				strcpy(to_send, "T");
				strcat(to_send, "1/1");
				strcat(to_send, ";");
				userSwitch = true;
			} else if (torque_temp1 == 0 && torque_temp2 > 0)
			{
				strcpy(to_send, "T");
				strcat(to_send, "0/1");
				strcat(to_send, ";");
				userSwitch = false;
			} else if (torque_temp1 > 0 && torque_temp2 == 0)
			{
				strcpy(to_send, "T");
				strcat(to_send, "1/0");
				strcat(to_send, ";");
				userSwitch = false;
			} else 
			{
				strcpy(to_send, "T");
				strcat(to_send, "0/0");
				strcat(to_send, ";");
				userSwitch = false;
			}
		}

		if (dt > 0.1)
		{
			if (sp->WriteData(to_send, strlen(to_send)))
			{
				printf("Sent %s\n", to_send);
			} else 
			{
				printf("Force data %s could not be sent\n", to_send);
			}
		}

        // if the tool is currently grasping an object we simply update the interaction grasp force
        // between the tool and the object (modeled by a virtual spring)
        if (graspActive && userSwitch)
        {
            // retrieve latest position and orientation of grasped ODE object in world coordinates
            cMatrix3d globalGraspObjectRot = graspObject->getGlobalRot();
            cVector3d globalGraspObjectPos = graspObject->getGlobalPos();

            // compute the position of the grasp point on object in global coordinates
            cVector3d globalGraspPos = globalGraspObjectPos + cMul(globalGraspObjectRot, graspPos);

            // retrieve the position of the tool in global coordinates
			cVector3d globalToolPos  = (index_finger->getProxyGlobalPos()+thumb->getProxyGlobalPos())/2;

            // compute the offset between the tool and grasp point on the object
           /*  cVector3d offset = (globalToolPos - globalGraspObjectPos)*0.1;
            // model a spring between both points
            double STIFFNESS = 50;
	 		double damper = 1000;

            cVector3d force; //STIFFNESS * dpos;
			//if (i == 10)
			//{
			if (previous_object_pos.x != 0 && previous_object_pos.y != 0 && previous_object_pos.z != 0)
			{
				force.x = -STIFFNESS*(globalGraspPos.x-(globalToolPos.x))-damper*(globalGraspPos.x-previous_object_pos.x);
				force.y = -STIFFNESS*(globalGraspPos.y-(globalToolPos.y))-damper*(globalGraspPos.y-previous_object_pos.y);
				force.z = -STIFFNESS*(globalGraspPos.z-(globalToolPos.z))-damper*(globalGraspPos.z-previous_object_pos.z);
					
				if (force.x > 10)
					force.x = 10;
				else if (force.x < -10)
					force.x = -10;

				if (force.y > 10)
					force.y = 10;
				else if (force.y < -10)
					force.y = -10;

				if (force.z > 10)
					force.z = 10;
				else if (force.z < -10)
					force.z = -10;

				//force.z += 9.81*graspObject->getMass();
				// apply attraction force (grasp) onto object
				//graspObject->addGlobalForceAtGlobalPos(force, globalGraspObjectPos);
			}*/
			graspObject->disableDynamics();
			graspObject->setPosition(globalToolPos);
			
			previous_object_pos = globalGraspPos;
            graspLine->m_pointA = globalGraspPos;
            graspLine->m_pointB = globalToolPos;

			cVector3d force;
			force.x = 0;
			force.y = 0;
			force.z = -9.81*graspObject->getMass();

			tool->m_lastComputedGlobalForce += force;
			//}

        }

        // the user is not or no longer currently grasping the object
        else
        {
            // was the user grasping the object at the previous simulation loop
            if (graspActive)
            {
                // we disable grasping
                graspActive = false;

                // we hide the virtual line between the tool and the grasp point
                graspLine->setShowEnabled(false);

                // we enable haptics interaction between the tool and the previously grasped object
                if (graspObject != NULL)
                {
                    graspObject->m_imageModel->setHapticEnabled(true, true);
					graspObject->enableDynamics();
                }
            }

            // the user is touching an object
            if (object_tool != NULL)
            {
                // check if object is attached to an external ODE parent
                cGenericType* externalParent = object_tool->getExternalParent();
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(externalParent);
                if (ODEobject != NULL)
                {
                    // get position of tool
                    cVector3d pos = tool->m_proxyPointForceModel->m_contactPoint0->m_globalPos;

                    // retrieve the haptic interaction force being applied to the tool
                    cVector3d force = tool->m_lastComputedGlobalForce;

                    // apply haptic force to ODE object
                    cVector3d tmpfrc = cNegate(force);
                    ODEobject->addGlobalForceAtGlobalPos(tmpfrc, pos);
                }
            }

            // the user is touching an object
            if (object_index_finger != NULL && object_thumb != NULL)
            {
                // check if object is attached to an external ODE parent
                cGenericType* externalParent = object_index_finger->getExternalParent();
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(externalParent);
                if (ODEobject != NULL)
                {
                    // get position of tool
                    cVector3d pos = index_finger->m_proxyPointForceModel->m_contactPoint0->m_globalPos;

			        // check if user has enabled the user switch to gras the object
                    if (userSwitch)
                    {
                        // a new object is being grasped
                        graspObject = ODEobject;

                        // retrieve the grasp position on the object in local coordinates
                        graspPos = (index_finger->m_proxyPointForceModel->m_contactPoint0->m_localPos + thumb->m_proxyPointForceModel->m_contactPoint0->m_localPos)/2;

                        // grasp in now active!
                        graspActive = true;

                        // enable small line which display the offset between the tool and the grasp point
                     //   graspLine->setShowEnabled(true);

                        // disable haptic interaction between the tool and the grasped device.
                        // this is performed for stability reasons.
                        graspObject->m_imageModel->setHapticEnabled(false, true);

						holding_pos1 = overall_pos1;
						holding_pos2 = overall_pos2;
                    }

                    // retrieve the haptic interaction force being applied to the tool
                    cVector3d force = index_finger->m_lastComputedGlobalForce;

                    // apply haptic force to ODE object
                    cVector3d tmpfrc = cNegate(force);
                    ODEobject->addGlobalForceAtGlobalPos(tmpfrc, pos);

					force = thumb->m_lastComputedGlobalForce;
					
					pos = thumb->m_proxyPointForceModel->m_contactPoint0->m_globalPos;

					// apply haptic force to ODE object
                    tmpfrc = cNegate(force);
                    ODEobject->addGlobalForceAtGlobalPos(tmpfrc, pos);
                }
            } else if ( object_index_finger != NULL)
			{
                // check if object is attached to an external ODE parent
                cGenericType* externalParent = object_index_finger->getExternalParent();
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(externalParent);
                if (ODEobject != NULL)
                {
                    // get position of tool
                    cVector3d pos = index_finger->m_proxyPointForceModel->m_contactPoint0->m_globalPos;

                    // retrieve the haptic interaction force being applied to the tool
                    cVector3d force = index_finger->m_lastComputedGlobalForce;

                    // apply haptic force to ODE object
                    cVector3d tmpfrc = cNegate(force);
                    ODEobject->addGlobalForceAtGlobalPos(tmpfrc, pos);
                }
			}
			else if (object_thumb != NULL)
			{
                // check if object is attached to an external ODE parent
                cGenericType* externalParent = object_thumb->getExternalParent();
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(externalParent);
                if (ODEobject != NULL)
                {
                    // get position of tool
                    cVector3d pos = thumb->m_proxyPointForceModel->m_contactPoint0->m_globalPos;

                    // retrieve the haptic interaction force being applied to the tool
                    cVector3d force = thumb->m_lastComputedGlobalForce;

                    // apply haptic force to ODE object
                    cVector3d tmpfrc = cNegate(force);
                    ODEobject->addGlobalForceAtGlobalPos(tmpfrc, pos);
                }
			}

        }

        // send forces to device
        tool->applyForces();

        // retrieve simulation time and compute next interval
        double time = simClock.getCurrentTimeSeconds();
        double nextSimInterval = cClamp(time, 0.0001, 0.001);

        // reset clock
        simClock.reset();
        simClock.start();

        // update simulation
        ODEWorld->updateDynamics(nextSimInterval);
		/*
		dpos.x = (tool->getProxyGlobalPos().x - previous_pos.x);
		dpos.y = (tool->getProxyGlobalPos().y - previous_pos.y);
		dpos.z = (tool->getProxyGlobalPos().z - previous_pos.z);

		if ( i == 10 )
		{
		previous_pos.x =  tool->getProxyGlobalPos().x;
 		previous_pos.y =  tool->getProxyGlobalPos().y;
		previous_pos.z =  tool->getProxyGlobalPos().z;
		i = 0;
		}
		else 
			i++;*/
		index_tool_line->m_pointA = index_finger->getProxyGlobalPos();
		index_tool_line->m_pointB = tool->getProxyGlobalPos();
		thumb_tool_line->m_pointA = thumb->getProxyGlobalPos();
		thumb_tool_line->m_pointB = tool->getProxyGlobalPos();
	//	Sleep(80);
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------

void createCube(cMesh* a_mesh, double a_size)
{
    const double HALFSIZE = a_size / 2.0;
    int vertices [6][6];

    // face -x
    vertices[0][0] = a_mesh->newVertex(-HALFSIZE,  HALFSIZE, -HALFSIZE);
    vertices[0][1] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[0][2] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE,  HALFSIZE);
    vertices[0][3] = a_mesh->newVertex(-HALFSIZE,  HALFSIZE,  HALFSIZE);

    // face +x
    vertices[1][0] = a_mesh->newVertex( HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[1][1] = a_mesh->newVertex( HALFSIZE,  HALFSIZE, -HALFSIZE);
    vertices[1][2] = a_mesh->newVertex( HALFSIZE,  HALFSIZE,  HALFSIZE);
    vertices[1][3] = a_mesh->newVertex( HALFSIZE, -HALFSIZE,  HALFSIZE);

    // face -y
    vertices[2][0] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[2][1] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[2][2] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE,  HALFSIZE);
    vertices[2][3] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE,  HALFSIZE);

    // face +y
    vertices[3][0] = a_mesh->newVertex( HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[3][1] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[3][2] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[3][3] = a_mesh->newVertex( HALFSIZE,   HALFSIZE,  HALFSIZE);

    // face -z
    vertices[4][0] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[4][1] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[4][2] = a_mesh->newVertex( HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[4][3] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE, -HALFSIZE);

    // face +z
    vertices[5][0] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE,  HALFSIZE);
    vertices[5][1] = a_mesh->newVertex( HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[5][2] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[5][3] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE,  HALFSIZE);

    // create triangles
    for (int i=0; i<6; i++)
    {
    a_mesh->newTriangle(vertices[i][0], vertices[i][1], vertices[i][2]);
    a_mesh->newTriangle(vertices[i][0], vertices[i][2], vertices[i][3]);
    }

    // set material properties to light gray
    a_mesh->m_material.m_ambient.set(0.5f, 0.5f, 0.5f, 1.0f);
    a_mesh->m_material.m_diffuse.set(0.7f, 0.7f, 0.7f, 1.0f);
    a_mesh->m_material.m_specular.set(1.0f, 1.0f, 1.0f, 1.0f);
    a_mesh->m_material.m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);

    // compute normals
    a_mesh->computeAllNormals();

    // compute collision detection algorithm
    a_mesh->createAABBCollisionDetector(1.01 * proxyRadius, true, false);
}
