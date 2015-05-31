#pragma once

#include <plugins/mmCalcMethod.h>

namespace mmClouds {
	////////////////////////////////////////////////////////////////////////////////
	/// Sample: extended version of invert point selection using mmCalcBlock
	/// objects. Both single and multi-threaded versions are available using
	/// the same code.  Method behavior is selected with constructor parameters
	/// 
	/// demonstrates:
	/// - point states
	/// - block processing (extended interface)
	/// - seamless multi-threaded processing
	////////////////////////////////////////////////////////////////////////////////
	class cmInvertSelectionEx : public mmCalcMethodEx
	{
	public: //ctor
		cmInvertSelectionEx(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces, bool m_bMultithreaded);

	private: //implementation
		void mmCalcMethod::Calculate();

	private: //members
	};

	class cmInvertBlock: public mmCalcBlock {
	public:
		cmInvertBlock(mmInt v_iBlockSize) {m_piStates = new mmInt[v_iBlockSize];}
		~cmInvertBlock(){delete [] m_piStates;}

		void ForEachBlock(mmInt p_iCloudID, mmInt p_iStartPoint, mmInt p_iBlockCount);

		mmInt* m_piStates;
	};
}