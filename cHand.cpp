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

// cHand authored by Edoardo Battaglia

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#include "cHand.h"
#include "chai3d.h"
#include <fstream>

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

cHand::cHand()
{

}

cHand::~cHand()
{
	for (int fingerid = 0; fingerid < n_fingers; fingerid++)
	{
		joint_transforms_container[fingerid].clear();
	}
	
	joint_transforms_container.clear();
	current_angles.clear();
}

void cHand::initialize(const std::vector<std::vector<cTransform>>& T, bool hasarcpalm)
{
	initialize_transforms(T, hasarcpalm);
	initialize_graphics();
}

void cHand::initialize_transforms(const std::vector<std::vector<cTransform>>& T, bool hasarcpalm)
{
	// T contains transforms describing the relative joint chain for all fingers (one chain for each finger)
	
	// 0.7 change: added implementation for archpalm joints
	// if hasarcpalm is true, the first joints of each finger from the second onward 
	// will be parents of the first joint of the following finger.

	t_vector_zero = T;
	
	n_fingers = (int)T.size(); 
	
	ndof = 0; 
	
	for (int fingerid = 0; fingerid < n_fingers; fingerid++)
	{
		ndof_finger[fingerid] = t_vector_zero[fingerid].size();
		ndof += ndof_finger[fingerid];
	}

	for (int ndofcounter = 0; ndofcounter < ndof; ndofcounter++)
	{
		current_angles.push_back(0.0f); 
	}

	// Initialization of std::vectors
	for (int fingerid = 0; fingerid < n_fingers; fingerid++)
	{
		joint_transforms_container.push_back(std::vector<cGenericObject*>());
		for (int fingerndofcounter = 0; fingerndofcounter < ndof_finger[fingerid]; fingerndofcounter++)
		{
			joint_transforms_container[fingerid].push_back(new cGenericObject);
		}		
	}

	for (int fingerid = 0; fingerid < n_fingers; fingerid++)
	{
		for (int fingerndofcounter = 0; fingerndofcounter < ndof_finger[fingerid]; fingerndofcounter++)
		{
			// First joints of each chain (i.e. finger)
			if (fingerndofcounter == 0) {
				if (hasarcpalm && fingerid > 2)
				{
					std::cout << fingerid << std::endl;
					joint_transforms_container[fingerid-1][fingerndofcounter]->addChild(
						joint_transforms_container[fingerid][fingerndofcounter]);
				}
				else
				{
					this->addChild(joint_transforms_container[fingerid][fingerndofcounter]);
				}
			}
			else
			{
				joint_transforms_container[fingerid][fingerndofcounter - 1]->addChild(joint_transforms_container[fingerid][fingerndofcounter]);
			}	

			joint_transforms_container[fingerid][fingerndofcounter]->setLocalTransform(
				t_vector_zero[fingerid][fingerndofcounter]);
		}
	}

	this->computeGlobalPositions(); //IMPORTANT! Without this the global positions and orientations are not updated
}

void cHand::initialize_graphics(cColorf & joints_color, cColorf & links_color, cColorf & arrows_color)
{
	if (joint_transforms_container.size() == 0)
	{
		std::cout << "Hand model kinematics not initialized! No graphics were drawn." << std::endl;
	}
	else if (!graphics_container.empty())
	{
		std::cout << "Graphics container not empty! No graphics were drawn." << std::endl;
	}
	else
	{
		// initialize std::vectors
		for (int fingerid = 0; fingerid < n_fingers; fingerid++)
		{
			graphics_container.push_back(std::vector<cMultiMesh*>());
			for (int fingerndofcounter = 0; fingerndofcounter < ndof_finger[fingerid]; fingerndofcounter++)
			{
				graphics_container[fingerid].push_back(new cMultiMesh);
			}

			axesArrows.push_back(std::vector<cMesh*>());
			for (int fingerndofcounter = 0; fingerndofcounter < ndof_finger[fingerid]; fingerndofcounter++)
			{
				axesArrows[fingerid].push_back(new cMesh());

				cCreateArrow(axesArrows[fingerid][fingerndofcounter]);
				axesArrows[fingerid][fingerndofcounter]->scaleXYZ(0.15, 0.15, 0.15);

				axesArrows[fingerid][fingerndofcounter]->setVertexColor(arrows_color);

				axesArrows[fingerid][fingerndofcounter]->setUseVertexColors(true);
			}

			palm_graphic_container = new cMultiMesh();

		}

		for (int fingerid = 0; fingerid < n_fingers; fingerid++)
		{
			for (int fingerndofcounter = 0; fingerndofcounter < ndof_finger[fingerid]; fingerndofcounter++)
			{
				joint_transforms_container[fingerid][fingerndofcounter]->addChild(graphics_container[fingerid][fingerndofcounter]);
				joint_transforms_container[fingerid][fingerndofcounter]->addChild(axesArrows[fingerid][fingerndofcounter]);
				
				graphics_container[fingerid][fingerndofcounter]->addMesh(new cMesh);
				// spheres are used to draw joints locations
				cCreateSphere(graphics_container[fingerid][fingerndofcounter]->getMesh(0), sphradius);
				graphics_container[fingerid][fingerndofcounter]->getMesh(0)->setVertexColor(joints_color);
				
				if (fingerndofcounter == 0) {
					this->addChild(joint_transforms_container[fingerid][fingerndofcounter]);
				}
				else
				{
					joint_transforms_container[fingerid][fingerndofcounter - 1]->addChild(
						joint_transforms_container[fingerid][fingerndofcounter]);

					graphics_container[fingerid][fingerndofcounter - 1]->addMesh(new cMesh);
					// cylinders are used for phalanxes
					cCreateCylinder(graphics_container[fingerid][fingerndofcounter - 1]->getMesh(1),
						joint_transforms_container[fingerid][fingerndofcounter]->getLocalTransform().getLocalPos().length(), 
						cylradius);
					
					graphics_container[fingerid][fingerndofcounter - 1]->getMesh(1)->setVertexColor(links_color);
					graphics_container[fingerid][fingerndofcounter - 1]->setUseVertexColors(true);

					// also rotate cylinder so that its axis [0 0 1] is mapped to the right direction
					cVector3d curdir = joint_transforms_container[fingerid][fingerndofcounter]->getLocalTransform().getLocalPos();
					curdir.normalize();
					graphics_container[fingerid][fingerndofcounter - 1]->getMesh(1)->setLocalRot(RotAtoB(cVector3d(0, 0, 1), curdir));
				}
			}

			// repeat for the last phalanx
			graphics_container[fingerid][ndof_finger[fingerid] - 1]->addMesh(new cMesh);
			
			cCreateCylinder(graphics_container[fingerid][ndof_finger[fingerid] - 1]->getMesh(1),
				fingerratios[fingerid]*
					joint_transforms_container[fingerid][ndof_finger[fingerid] - 1]->
					getLocalTransform().getLocalPos().length(),
				cylradius);

			graphics_container[fingerid][ndof_finger[fingerid] - 1]->getMesh(1)->setVertexColor(links_color);
			
			cVector3d curdir = joint_transforms_container[fingerid][ndof_finger[fingerid] - 1]->getLocalTransform().getLocalPos();
			curdir.normalize();
			graphics_container[fingerid][ndof_finger[fingerid] - 1]->getMesh(1)->setLocalRot(RotAtoB(cVector3d(0, 0, 1), curdir));

			// add an extra sphere for the fingertip
			graphics_container[fingerid][ndof_finger[fingerid] - 1]->addMesh(new cMesh);
			
			cCreateSphere(graphics_container[fingerid][ndof_finger[fingerid] - 1]->getMesh(2),
				sphradius);
			graphics_container[fingerid][ndof_finger[fingerid] - 1]->getMesh(2)->setVertexColor(joints_color);
			graphics_container[fingerid][ndof_finger[fingerid] - 1]->setUseVertexColors(true);

			graphics_container[fingerid][ndof_finger[fingerid] - 1]->getMesh(2)->
				setLocalPos(fingerratios[fingerid] *
						joint_transforms_container[fingerid][ndof_finger[fingerid] - 1]->getLocalTransform().
						getLocalPos().length());
		}

		// create palm automatically (note: no arcpalm in the primitive-based visualization)
		for (int palmelid = 0; palmelid < n_fingers - 1; palmelid++)
		{
			cVector3d curdir = joint_transforms_container[palmelid + 1][0]->getLocalPos() -
				joint_transforms_container[palmelid][0]->getLocalPos();
			
			palm_graphic_container->addMesh(new cMesh);

			palm_graphic_container->getMesh(palmelid)->setLocalPos(joint_transforms_container[palmelid][0]->getLocalPos());
			cCreateCylinder(palm_graphic_container->getMesh(palmelid), curdir.length(), cylradius);
			
			palm_graphic_container->getMesh(palmelid)->setVertexColor(links_color);
			
			curdir.normalize();
			palm_graphic_container->getMesh(palmelid)->setLocalRot(RotAtoB(cVector3d(0, 0, 1), curdir));
		}

		cVector3d firstjoint = joint_transforms_container[0][0]->getLocalPos();
		cVector3d lastjoint = joint_transforms_container.back()[0]->getLocalPos();

		cVector3d closurepoint(firstjoint(0), lastjoint(1), firstjoint(2));

		cVector3d firsttocorner = closurepoint - firstjoint;

		palm_graphic_container->addMesh(new cMesh);
		palm_graphic_container->getMesh(n_fingers - 1)->setLocalPos(firstjoint);
		cCreateCylinder(palm_graphic_container->getMesh(n_fingers - 1), firsttocorner.length(), cylradius);
		palm_graphic_container->getMesh(n_fingers - 1)->setVertexColor(links_color);

		firsttocorner.normalize();
		palm_graphic_container->getMesh(n_fingers - 1)->setLocalRot(RotAtoB(cVector3d(0, 0, 1), firsttocorner));

		cVector3d lasttocorner = lastjoint - closurepoint;

		palm_graphic_container->addMesh(new cMesh);
		palm_graphic_container->getMesh(n_fingers)->setLocalPos(closurepoint);
		cCreateCylinder(palm_graphic_container->getMesh(n_fingers), lasttocorner.length(), cylradius);
		palm_graphic_container->getMesh(n_fingers)->setVertexColor(links_color);

		lasttocorner.normalize();
		palm_graphic_container->getMesh(n_fingers)->setLocalRot(RotAtoB(cVector3d(0, 0, 1), lasttocorner));

		palm_graphic_container->addMesh(new cMesh);
		cCreateSphere(palm_graphic_container->getMesh(n_fingers + 1), sphradius);
		palm_graphic_container->getMesh(n_fingers + 1)->setVertexColor(joints_color);

		palm_graphic_container->setUseVertexColors(true);

		palm_graphic_container->getMesh(n_fingers + 1)->setLocalPos(closurepoint);

		this->addChild(palm_graphic_container);

		//  toggle axes arrows off after initialization
		this->toggleArrows();
		this->setWireMode(false, true);
	}
}

void cHand::initialize_graphics(std::vector<std::string> mesh_files, std::vector<cTransform> localTransform, 
	std::vector<int> mesh_map, cColorf & arrows_color)
{
	// mesh_files -> file names for the stl to load
	// mesh_map -> vector of ids of joints that are parents to each stl. 0 = base frame
	
	int meshfilecounter = 0;
	int jointcounter = 0;
	
	// first of all, check if the first element of mesh_map is 0. 
	// If it is load the first mesh in to palm container

	if (mesh_map[meshfilecounter] == 0)
	{
		palm_graphic_container = new cMultiMesh;
		this->addChild(palm_graphic_container);

		bool fileload = false;
		fileload = palm_graphic_container->loadFromFile(mesh_files[meshfilecounter+1]);

		if (!fileload)
		{
			printf("Error - 3D Model failed to load correctly.\n");
		}

		int hasbasemesh = 0;
		meshfilecounter++;
	}

	// then`initialize a graphics container for each joint
		
	for (int fingerid = 0; fingerid < n_fingers; fingerid++)
	{
		graphics_container.push_back(std::vector<cMultiMesh*>());
		for (int fingerndofcounter = 0; fingerndofcounter < ndof_finger[fingerid]; fingerndofcounter++)
		{
			graphics_container[fingerid].push_back(new cMultiMesh);
		}

		axesArrows.push_back(std::vector<cMesh*>());
		for (int fingerndofcounter = 0; fingerndofcounter < ndof_finger[fingerid]; fingerndofcounter++)
		{
			axesArrows[fingerid].push_back(new cMesh());

			cCreateArrow(axesArrows[fingerid][fingerndofcounter]);
			axesArrows[fingerid][fingerndofcounter]->scaleXYZ(0.15, 0.15, 0.15);

			axesArrows[fingerid][fingerndofcounter]->setVertexColor(arrows_color);

			axesArrows[fingerid][fingerndofcounter]->setUseVertexColors(true);
		}
	}
	
	// finally load all other files by matching them with the appropriate joint container
	
	for (int fingerid = 0; fingerid < n_fingers; fingerid++)
	{
		for (int fingerndofcounter = 0; fingerndofcounter < ndof_finger[fingerid]; fingerndofcounter++)
		{
			joint_transforms_container[fingerid][fingerndofcounter]->addChild(axesArrows[fingerid][fingerndofcounter]);
			joint_transforms_container[fingerid][fingerndofcounter]->addChild(
				graphics_container[fingerid][fingerndofcounter]);
			
			if (mesh_map[meshfilecounter] == jointcounter + 1) // note: 0 is the base mesh, 
			{
				bool fileload;
				
				fileload = graphics_container[fingerid][fingerndofcounter]->loadFromFile(
					mesh_files[meshfilecounter]);

				if (!fileload)
				{
					printf("Error - 3D Model failed to load correctly.\n");
				}
				
				graphics_container[fingerid][fingerndofcounter]->setLocalTransform(localTransform[meshfilecounter]);

				this->computeGlobalPositions();

				meshfilecounter++;
			}
			jointcounter++;
		}
	}

	this->toggleArrows();

	this->setWireMode(false, true);

	this->computeGlobalPositions();
}

std::vector<std::vector<cVector3d*>> cHand::getHandCenters()
{
	std::vector<std::vector<cVector3d*>> centerPos;

	for (int fingerid = 0; fingerid < n_fingers; fingerid++)
	{
		centerPos.push_back(std::vector<cVector3d*>());
		for (int fingerndofcounter = 0; fingerndofcounter < ndof_finger[fingerid]; fingerndofcounter++)
		{
			centerPos[fingerid].push_back(new cVector3d);

			if (fingerndofcounter == ndof_finger[fingerid] - 1)
			{// add an additional element for the fingertip
				centerPos[fingerid].push_back(new cVector3d);
			}
		}
	}

	for (int fingerid = 0; fingerid < n_fingers; fingerid++)
	{
		for (int fingerndofcounter = 0; fingerndofcounter < ndof_finger[fingerid]; fingerndofcounter++)
		{
			*centerPos[fingerid][fingerndofcounter] = 
				graphics_container[fingerid][fingerndofcounter]->getMesh(0)->getGlobalPos();
			//std::cout << *centerPos[fingerid][fingerndofcounter] << std::endl;

			if (fingerndofcounter == ndof_finger[fingerid] - 1)
			{// add an additional element for the fingertip
				*centerPos[fingerid][fingerndofcounter+1] =
					graphics_container[fingerid][fingerndofcounter]->getMesh(2)->getGlobalPos();
			}
		}
	}

	return(centerPos);
}

void cHand::updateAngles(const std::vector<double> newangles)
{
	if (newangles.size() == current_angles.size())
	{
		current_angles = newangles;
	}
	else
	{
		std::cout << "Warning, angles vector dimension mismatch. No assignment was done." << std::endl;
	}
}

void cHand::toggleArrows()
{// show/hide axes arrows, and toggle wireframe mode for the hand model to be able to see through
	this->setWireMode(!this->getWireMode(), true);

	for (int fingerid = 0; fingerid < n_fingers; fingerid++)
	{
		for (int fingerndofcounter = 0; fingerndofcounter < ndof_finger[fingerid]; fingerndofcounter++)
		{
			axesArrows[fingerid][fingerndofcounter]->setEnabled(!axesArrows[fingerid][fingerndofcounter]->getEnabled());
			axesArrows[fingerid][fingerndofcounter]->setWireMode(false);
		}
	}
}

void cHand::updateKinematics()
{// Update visualization according to the values of joint angles.
	int jointcount = 0;

	for (int fingerid = 0; fingerid < n_fingers; fingerid++)
	{
		for (int fingerndofcounter = 0; fingerndofcounter < ndof_finger[fingerid]; fingerndofcounter++)
		{
			cVector3d rotaxis(0, 0, 1);
			
			cMatrix3d tmpRot = cMatrix3d(rotaxis, current_angles[jointcount]);
			cVector3d tmpPos(0,0,0);
			
			cTransform tmpT = cTransform(tmpPos, tmpRot);

			joint_transforms_container[fingerid][fingerndofcounter]->setLocalTransform(
				t_vector_zero[fingerid][fingerndofcounter]*tmpT);

			jointcount++;
		}
	}
	this->computeGlobalPositions();
}

std::vector<std::vector<cTransform>>  cHand::makeTFromFile(std::string filename, std::vector<int> dof_list)
{// load a collection of transforms from file filename. dof_list is a list of integers stating how many dof are in each finger
// NOTE: if the hand model has arcpalm dof those should be counted as dof for the finger that is immediately distal 
	std::vector<std::vector<cTransform>> Tfromfile;
	std::vector<cTransform> Tbuffer;
	cTransform Tbuffersinglematrix;
	std::vector<double> Tbufferelems;

	std::ifstream myReadFile;
	myReadFile.open(filename);
	std::string output;


	if (myReadFile.is_open()) {
			for (int dofid = 0; dofid < (int)dof_list.size(); dofid++) 
			{// for each finger
				for (int dofcounter = 0; dofcounter < dof_list[dofid]; dofcounter++)
				{// for each dof for current finger
					for (int rows = 0; rows < 4; rows++)
					{
						for (int cols = 0; cols < 3; cols++)
						{
							getline(myReadFile, output, ' ');
							
							Tbufferelems.push_back(std::stod(output));
							
						}
						getline(myReadFile, output, '\n');
						
						Tbufferelems.push_back(std::stod(output));
						
					}
				
					Tbuffersinglematrix.set(
						cVector3d(Tbufferelems[3], Tbufferelems[7], Tbufferelems[11]),
						cMatrix3d(Tbufferelems[0], Tbufferelems[1], Tbufferelems[2],
							Tbufferelems[4], Tbufferelems[5], Tbufferelems[6],
							Tbufferelems[8], Tbufferelems[9], Tbufferelems[10])
						);
				
					Tbuffer.push_back(cTransform(Tbuffersinglematrix));

					Tbuffersinglematrix.identity();
					Tbufferelems.clear();
					
				}
				
				Tfromfile.push_back(Tbuffer);
				Tbuffer.clear();
			}
			
	}
	else
	{
		std::cout << "cHand.makeTFromFile(filename, doflist): File not found!" << std::endl;
	}
	
	myReadFile.close();

	return(Tfromfile);
	
}

cMatrix3d cHand::RotAtoB(const cVector3d& a, const cVector3d& b)
{
	// calculate a matrix mapping a to b, derived from Rodrigues' rotation formula and
	// the fact that mapping a vector a to a vector b is the same as rotating a around 
	// (a+b)/2 by 180 degrees (i.e. mirror a around (a+b)/2).

	cVector3d absum = a + b;
	cMatrix3d I;
	I.identity();

	cMatrix3d R;

	R.setCol(absum*absum(0), absum*absum(1), absum*absum(2));

	R *= 2/(absum.dot(absum));
	R -= I;

	return(R);
}

void cHand::printcTransform(cTransform T, std::string Tname)
{
	std::cout << Tname << ": " << std::endl;
	cTransform dummy = T;
	std::cout << dummy(0, 0) << " " << dummy(0, 1) << " " << dummy(0, 2) << " " << dummy(0, 3) << " " << std::endl;
	std::cout << dummy(1, 0) << " " << dummy(1, 1) << " " << dummy(1, 2) << " " << dummy(1, 3) << " " << std::endl;
	std::cout << dummy(2, 0) << " " << dummy(2, 1) << " " << dummy(2, 2) << " " << dummy(2, 3) << " " << std::endl;
	std::cout << dummy(3, 0) << " " << dummy(3, 1) << " " << dummy(3, 2) << " " << dummy(3, 3) << " " << std::endl;
}

void cHand::printcMatrix3d(cMatrix3d M, std::string Mname)
{
	std::cout << Mname << ": " << std::endl;
	cMatrix3d dummy = M;
	std::cout << dummy(0, 0) << " " << dummy(0, 1) << " " << dummy(0, 2) << " " << std::endl;
	std::cout << dummy(1, 0) << " " << dummy(1, 1) << " " << dummy(1, 2) << " " << std::endl;
	std::cout << dummy(2, 0) << " " << dummy(2, 1) << " " << dummy(2, 2) << " " << std::endl;
}


void cHand::printcVector3d(cVector3d V, std::string Vname)
{
	std::cout << Vname << ": " << std::endl;
	cVector3d dummy = V;
	std::cout << dummy(0) << " " << dummy(1) << " " << dummy(2) << " " << std::endl;
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
