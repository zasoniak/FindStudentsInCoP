#pragma once

#include <plugins/mmCalcMethod.h>

namespace mmClouds {
	////////////////////////////////////////////////////////////////////////////////
	/// Empty calculation method
	/// TODO: provide documentation
	////////////////////////////////////////////////////////////////////////////////
	class cmEmptyMethod : public mmCalcMethod
	{
	public: //ctor
		cmEmptyMethod(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces);

	private: //implementation
		void mmCalcMethod::Calculate();

	private: //members
	};
}