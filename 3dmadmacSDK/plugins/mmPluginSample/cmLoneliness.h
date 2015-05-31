#pragma once

#include <plugins/mmCalcMethod.h>

namespace mmClouds {
	////////////////////////////////////////////////////////////////////////////////
	/// Sample: Algorithm creating additional data layer with nearest neighbor
	/// distance from every point
	/// 
	/// demonstrates:
	/// - creating layers
	/// - extended interface
	/// - block processing
	/// - ForEachCloud events
	/// - thread id disambiguation
	////////////////////////////////////////////////////////////////////////////////
	class cmLoneliness : public mmCalcMethodEx, public mmCalcBlock
	{
	public: //ctor
		cmLoneliness(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces);

	private: //implementation
		void mmCalcMethod::Calculate();
		void mmCalcBlock::ForEachBlock( mmInt p_iCloudID, mmInt p_iFirstPoint, mmInt p_iPointsCount);

		void OnInitialize();
		void OnFinish();

	private: //members

		mmString m_sDLName;
		mmInt m_iDLID;

		std::map<mmInt, mmMath::sPoint3D*> m_mCoordBuffers;
		std::map<mmInt, mmReal*> m_mDLBuffers;
	};
}