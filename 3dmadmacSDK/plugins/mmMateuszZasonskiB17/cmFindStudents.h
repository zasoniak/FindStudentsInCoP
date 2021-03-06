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
		mmReal m_rSegmentationRadius = 25.0;
		mmReal m_rScale;
		mmString m_sCloudName;
		mmReal m_rPlaneDeletionRadius = 50.0;
		mmReal m_rPlaneDeletionMaxError = 0.3;
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
		bool *m_bPlanesDetecion;	// flag if a point is a part of a plane

		void clearCloudOfPointsFromSurfaceEdges();

		void segmentateCloudOfPoints();	//group points using Hausdorf distance
		mmReal *m_rGroupNumber;
		mmInt m_igroupNumber;
		mmInt *m_iGroupMembersCount;
		void mergeSegments();

		void detectFacesByHSV();	//coarse people detection using HSV values to detect faces
		mmReal *m_rGroupsMarkedAsHuman;

		void detectHumansByGeometry();	//people detection by find most complicated geometry

		void saveCloudOfPoints();	//saving cloud of points

		void saveResultsToFile();	//exporting results to txt file


		void RGBtoHSV(mmReal r, mmReal g, mmReal b, mmReal *h, mmReal *s, mmReal *v);


	};
}