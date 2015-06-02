#pragma once

#include <plugins/mmCalcMethod.h>
#include <math.h>

namespace mmClouds {
	////////////////////////////////////////////////////////////////////////////////
	/// Empty calculation method
	/// TODO: provide documentation
	////////////////////////////////////////////////////////////////////////////////
	class cmCheckHSV : public mmCalcMethod
	{
	public:
		cmCheckHSV(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces);	//constructor

		/*
		members with values provided by user
		*/

		mmString m_sCloudName;

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


		void checkHSV();	//coarse people detection using HSV values to detect faces
		void checkSphere();


		void RGBtoHSV(mmReal r, mmReal g, mmReal b, mmReal *h, mmReal *s, mmReal *v);


	};
}