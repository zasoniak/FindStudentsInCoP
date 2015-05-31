#pragma once

#include <plugins/mmCalcMethod.h>

namespace mmClouds {
	////////////////////////////////////////////////////////////////////////////////
	/// Empty calculation method
	/// TODO: provide documentation
	////////////////////////////////////////////////////////////////////////////////
	class cmAverageNormalVector : public mmCalcMethod
	{
	public: //ctor
		cmAverageNormalVector(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces);
		mmReal m_rRadius, m_rScale;
		mmString m_sCloudName;
		mmReal m_sRadius =1.0;
	private: //implementation
		void mmCalcMethod::Calculate();

	private: //members
		mmInt m_iID;		//id chmury


	};
}