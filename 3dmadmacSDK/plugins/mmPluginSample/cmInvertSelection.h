#pragma once

#include <plugins/mmCalcMethod.h>

namespace mmClouds {

	////////////////////////////////////////////////////////////////////////////////
	/// Sample: calculation method for inverting point selection.
	/// 
	/// demonstrates:
	/// - point states
	/// - memory management
	/// - points-per-pass approach
	/// - test stop execution
	/// - running pointer
	////////////////////////////////////////////////////////////////////////////////
	class cmInvertSelection : public mmCalcMethod
	{
	public: //ctor, dtor
		cmInvertSelection(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces);
		~cmInvertSelection(void);

	private: //implementation
		void mmCalcMethod::Calculate();

	private: //members
	};
}
