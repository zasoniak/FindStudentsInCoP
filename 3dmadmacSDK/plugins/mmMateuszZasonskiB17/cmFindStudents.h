#pragma once

#include <plugins/mmCalcMethod.h>
#include <math.h>

namespace mmClouds {
	////////////////////////////////////////////////////////////////////////////////
	/// Empty calculation method
	/// TODO: provide documentation
	////////////////////////////////////////////////////////////////////////////////
	class cmFindStudents : public mmCalcMethod
	{
	public: 
		cmFindStudents(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces);	//constructor
		
		/*
		members with values provided by user
		*/
		mmReal m_rSegmentationRadius = 10.0;
		mmReal m_rScale;
		mmString m_sCloudName;
		mmReal m_rPlaneDeletionRadius = 10.0;
		mmReal m_rPlaneDeletionMaxError = 0.5;
	private: 
		void mmCalcMethod::Calculate();	//entry point for calculation

		mmInt m_iID;		//id chmury
		mmInt m_iPointsCount;
		mmInt m_iPointStart;


		void sortCloudOfPoints();	//sorting existing clouds of points
		
		void loadCloudOfPoints();		// loading cloud of points provided by user 
		/*
		members initialized with loading cloud of points
		*/
		mmReal *m_rGlobalXYZ;
		mmReal *m_rNXNYNZ;
		mmInt *m_iPointsState;
		unsigned char* m_ucRGBA;


		void clearCloudOfPointsFromLargeSurfaces();	//marks large surfaces (like tables, walls) as deleted
		mmInt *m_iPlanesDetecion;	// flag if a point is a part of a plane

		void segmentateCloudOfPoints();	//group points using Hausdorf distance
		mmReal *m_rGroupNumber;
		mmInt *m_iGroupMembersCount;

		void detectFacesByHSV();	//coarse people detection using HSV values to detect faces

		void saveCloudOfPoints();	//saving cloud of points

		void saveResultsToFile();	//exporting results to txt file


	};
}