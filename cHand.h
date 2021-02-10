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
#ifndef cHandH
#define cHandH
//------------------------------------------------------------------------------
//#include <fstream>
//#include <string>
#include "world/CGenericObject.h"
#include "world/CMultiMesh.h"
#include "world/CShapeSphere.h"
#include "world/CShapeCylinder.h"
#include "materials/CMaterial.h"
#include "materials/CTexture2d.h"
//------------------------------------------------------------------------------
//

namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
	\file       cHand.h

	\brief
	Implementation of a hand visualization system.
*/
//==============================================================================

//==============================================================================
/*!
	\class      cHand
	\ingroup    world

	\brief
	This class implements a hand visualizer.

	\details
	This class implements a hand visualizer.
*/
//==============================================================================
	class cHand : public cGenericObject
	{

	public:
		//--------------------------------------------------------------------------
		// CONSTRUCTOR & DESTRUCTOR:
		//--------------------------------------------------------------------------

		//! Constructor of cHand.
		cHand();

		//! Destructor of cHand.
		virtual ~cHand();

		//--------------------------------------------------------------------------
		// PUBLIC METHODS:
		//--------------------------------------------------------------------------
		//void LoadFromFile();
		//
	public:

		//! build a collection of transforms from a plain .txt file. Assumes a plain txt file where every row has 4 matrix components.
		std::vector<std::vector<cTransform>>  makeTFromFile(std::string filename, std::vector<int> dof_list);

		//! defines hand kinematics (joints position and orientation) as defined by a collection of cTransforms
		void initialize_transforms(const std::vector<std::vector<cTransform>>&, bool hasarcpalm = false);
		
		void initialize_graphics(cColorf& joints_color = cColorf(32.0 / 255.0, 3.0 / 255.0, 113.0 / 255.0, 1.0),
			//cColorf& links_color  = cColorf(122.0/255.0, 218.0 / 255.0, 192.0 / 255.0, 1.0),
			cColorf& links_color = cColorf(0.0, 0.5, 0.5, 1.0),
			cColorf & arrows_color = cColorf(0.0, 0.0, 0.8, 1.0));
		
		//! creates graphics from a list of STL files
		void initialize_graphics(std::vector<std::string>, std::vector<cTransform>, std::vector<int>,
			cColorf & arrows_color = cColorf(0.0, 0.0, 0.8, 1.0));

		//! get positions of all center of rotations plus fingertips
		std::vector<std::vector<cVector3d*>> getHandCenters();

		//! initialization for both transforms and graphics (uses the primitive-based initialization) 
		void initialize(const std::vector<std::vector<cTransform>>&, bool hasarcpalm = false);

		//! default initializations from some premade hand models
		void initialize_Simple20Dof() { initialize(t_default_Simple20DoF); };
		void initialize_Tkach25Dof() { initialize(t_default_Tkach25Dof); };

		//! assign new values to joint angles
		void updateAngles(const std::vector<double>);
		
		//! update visualization to reflect changes on joint angles
		void updateKinematics();

		//! toggle visualization of arrows for directional axes
		void toggleArrows();

		//! Access total number of degrees of freedom
		inline int getdof() { return this->ndof; }

		//! Access total number of fingers
		inline int getnfingers() { return this->n_fingers; }

		//! Access number of degrees of freedom for one finger
		inline int getdof_finger(int id) { return this->ndof_finger[id]; }



		//! function to print a cTransform
		void printcTransform(cTransform T, std::string Tname);
		//! function to print a cMatrix3d
		void printcMatrix3d(cMatrix3d M, std::string Mname);
		//! function to print a cVector3d
		void printcVector3d(cVector3d V, std::string Vname);

	//--------------------------------------------------------------------------
	// PROTECTED METHODS:
	//--------------------------------------------------------------------------
	
protected:
	
	//! computes the matrix mapping vector a to vector b.
	cMatrix3d RotAtoB(const cVector3d& a, const cVector3d& b);
	
	//--------------------------------------------------------------------------
	// PROTECTED MEMBERS: (cHand)
	//--------------------------------------------------------------------------

protected:

	//! number of fingers for the hand model.
	int n_fingers;

	//! number of degrees of freedom for the hand model.
	int ndof;

	//! number of degrees of freedom for each finger.
	int ndof_finger[100];
	
	//! joint angles buffer.
	std::vector<double> current_angles;

	//! ratio between last phalanx and previous phalanx lenght
	// by default it's values from Buryanov & Kotiuk 2010 "Proportions of Hand Segments"
	std::vector<double> fingerratios = {0.68, 0.7, 0.66, 0.67, 0.88};

	//! finger thickness.
	int finger_thickness = 10;

	//! parameters for joint visualization
	double sphradius = 0.9*0.01;
	double cylradius = 0.9*0.0065;

	/**** std::vectors to store graphics. ****/
	//! stores the transformation data, commands kinematics
	std::vector<std::vector<cGenericObject*>> joint_transforms_container; 
	//! stores all the graphics
	std::vector<std::vector<cMultiMesh*>> graphics_container;
	//! graphics for axes of rotation
	std::vector<std::vector<cMesh*>> axesArrows; // 
	//! contains graphics for visualization of the palm
	cMultiMesh* palm_graphic_container;

	//! container for transforms describing joints relative pose in the zero pose.
	std::vector<std::vector<cTransform>> t_vector_zero;

	//! Pi, needed for forward kinematics
	double Pi = 3.14159;

public:
	//! default cTransform collection for the Tkach hand model
	// Anastasia Tkach, Andrea Tagliasacchi, Edoardo Remelli, Mark Pauly, and Andrew Fitzgibbon. 2017. 
	// Online Generative Model Personalization for Hand Tracking. ACM Trans. Graph. 36, 6 (2017).
	std::vector<std::vector<cTransform>> t_default_Tkach25Dof =
	{
		/********************************************************/
		// Thumb
		{cTransform(
			cVector3d(-0.05f, 0.045f, -0.0f), cMatrix3d(cVector3d(1, 0, 0), 0) // abduction z
			),
		cTransform(
			cVector3d(0.00f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), -90 * (Pi / 180))  // flexion x
			),
		cTransform(
			cVector3d(0.00f, 0.0f, 0.0f), cMatrix3d(cVector3d(0, 1, 0), 90 * (Pi / 180))  // twist y
			),
		cTransform(
			cVector3d(0.00f, 0.0f, 0.05f), cMatrix3d(cVector3d(0, 1, 0), -90 * (Pi / 180))*cMatrix3d(cVector3d(1, 0, 0), 90 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 0)
			)
		},
		/********************************************************/
		// Index
		{cTransform(
			cVector3d(0.0f, 0.025f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 0) // abduction z
			),
		cTransform(
			cVector3d(0.00f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), -90 * (Pi / 180))  // flexion x
			),
		cTransform(
			cVector3d(0.00f, 0.0f, 0.0f), cMatrix3d(cVector3d(0, 1, 0), 90 * (Pi / 180))  // twist y
			),
		cTransform(
			cVector3d(0.0f, 0.0f, 0.05f), cMatrix3d(cVector3d(0, 1, 0), -90 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f),cMatrix3d(cVector3d(1, 0, 0), 0 * (Pi / 180))
			)
		},
		/********************************************************/
		// Middle
		{cTransform(
			cVector3d(0.0f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 0) // abduction z
			),
		cTransform(
			cVector3d(0.00f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), -90 * (Pi / 180))  // flexion x
			),
		cTransform(
			cVector3d(0.00f, 0.0f, 0.0f), cMatrix3d(cVector3d(0, 1, 0), 90 * (Pi / 180))  // twist y
			),
		cTransform(
			cVector3d(0.0f, 0.0f, 0.05f), cMatrix3d(cVector3d(0, 1, 0), -90 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f),cMatrix3d(cVector3d(1, 0, 0), 0 * (Pi / 180))
			)
		},
		/********************************************************/
		// Ring
		{cTransform(
			cVector3d(0.0f, -0.025f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 0) // abduction z
			),
		cTransform(
			cVector3d(0.00f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), -90 * (Pi / 180))  // flexion x
			),
		cTransform(
			cVector3d(0.00f, 0.0f, 0.0f), cMatrix3d(cVector3d(0, 1, 0), 90 * (Pi / 180))  // twist y
			),
		cTransform(
			cVector3d(0.0f, 0.0f, 0.05f), cMatrix3d(cVector3d(0, 1, 0), -90 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f),cMatrix3d(cVector3d(1, 0, 0), 0 * (Pi / 180))
			)
		},
		/********************************************************/
		// Little
		{cTransform(
			cVector3d(0.0f, -0.05f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 0) // abduction z
			),
		cTransform(
			cVector3d(0.00f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), -90 * (Pi / 180))  // flexion x
			),
		cTransform(
			cVector3d(0.00f, 0.0f, 0.0f), cMatrix3d(cVector3d(0, 1, 0), 90 * (Pi / 180))  // twist y
			),
		cTransform(
			cVector3d(0.0f, 0.0f, 0.05f), cMatrix3d(cVector3d(0, 1, 0), -90 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f),cMatrix3d(cVector3d(1, 0, 0), 0 * (Pi / 180))
			)
		}
	};
	
	//! default cTransform collection for a simple 20dof hand model
	std::vector<std::vector<cTransform>> t_default_Simple20DoF = 
	{
		/********************************************************/
		// Thumb
		{cTransform(
			cVector3d(-0.05f, 0.045f, -0.0f), cMatrix3d(cVector3d(1, 0, 0), 0)
			),
		cTransform(
			cVector3d(0.00f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), -90 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 90 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f),cMatrix3d(cVector3d(1, 0, 0), 0)
			)
		},
		/********************************************************/
		// Index
		{cTransform(
			cVector3d(0.0f, 0.025f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 0)
			),
		cTransform(
			cVector3d(0.0f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), -90 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 0 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f),cMatrix3d(cVector3d(1, 0, 0), 0 * (Pi / 180))
			)
		},
		/********************************************************/
		// Middle
		{cTransform(
			cVector3d(0.0f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 0)
			),
		cTransform(
			cVector3d(0.0f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), -90 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 0 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f),cMatrix3d(cVector3d(1, 0, 0), 0 * (Pi / 180))
			)
		},
		/********************************************************/
		// Ring
		{cTransform(
			cVector3d(0.0f, -0.025f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 0)
			),
		cTransform(
			cVector3d(0.0f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), -90 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 0 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f),cMatrix3d(cVector3d(1, 0, 0), 0 * (Pi / 180))
			)
		},
		/********************************************************/
		// Little
		{cTransform(
			cVector3d(0.0f, -0.05f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 0)
			),
		cTransform(
			cVector3d(0.0f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), -90 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f), cMatrix3d(cVector3d(1, 0, 0), 0 * (Pi / 180))
			),
		cTransform(
			cVector3d(0.05f, 0.0f, 0.0f),cMatrix3d(cVector3d(1, 0, 0), 0 * (Pi / 180))
			)
		}

	};

};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------