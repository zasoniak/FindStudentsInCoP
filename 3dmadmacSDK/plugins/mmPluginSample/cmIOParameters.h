#pragma once

#include <plugins/mmCalcMethod.h>

namespace mmClouds {
	////////////////////////////////////////////////////////////////////////////////
	/// Sample: do-nothing method showing UI window with various parameter types
	/// and according descriptions
	/// 
	/// demonstrates:
	/// - input/output parameters
	/// - persistent (static) parameters
	/// - GUI options
	////////////////////////////////////////////////////////////////////////////////
	class cmIOParameters : public mmCalcMethod
	{
	public: //ctor
		cmIOParameters(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces);

	private: //implementation
		void mmCalcMethod::Calculate();

	private: //members

		bool m_bTest;
		mmInt m_iTest;
		mmReal m_rTest;
		mmString m_sTest;
		mmString m_sDLTest;
		mmString m_sCloudTest;
		static mmString m_sPersistentTest;

		mmInt m_iTestOutput;
		mmInt m_iTestInOut;
	};
}