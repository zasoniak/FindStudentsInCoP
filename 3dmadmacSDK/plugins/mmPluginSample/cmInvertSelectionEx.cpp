#include "cmInvertSelectionEx.h"



////////////////////////////////////////////////////////////////////////////////
/// Two calculation method signatures for both single- and multi-threaded
/// execution.
////////////////////////////////////////////////////////////////////////////////

static const
	mmClouds::mmCloudOfPointsCalculationMethodI::sCalculationMethodParams cmInvertSelectionExParamsST = {
		L"Sample: Invert point selection ST",
		L"{2D17641D-0F6D-41C2-8C97-7D4FEED38C41}",
		L"Finds complementary set of selected points (single-threaded version)",
		false // single-threaded
};

static const
	mmClouds::mmCloudOfPointsCalculationMethodI::sCalculationMethodParams cmInvertSelectionExParamsMT = {
		L"Sample: Invert point selection MT",
		L"{BAE0DCDD-1B37-4642-B816-50C38FD50104}",
		L"Finds complementary set of selected points (multi-threaded version)",
		true // multi-threaded
};

mmClouds::cmInvertSelectionEx::cmInvertSelectionEx(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces, bool m_bMultithreaded):
		mmCalcMethodEx(p_psGlobalInterfaces, m_bMultithreaded ? cmInvertSelectionExParamsMT : cmInvertSelectionExParamsST)
{
	FUNC_SCOPE;

	/*if (m_bMultithreaded)*/ BindParam(m_iPointsPerBlock, L"Points per block", Int);
}

void mmClouds::cmInvertSelectionEx::Calculate() 
{
	FUNC_SCOPE;

	ForEachCloud(std::tr1::shared_ptr<cmInvertBlock>(new cmInvertBlock(m_iPointsPerBlock)).get());
}

void mmClouds::cmInvertBlock::ForEachBlock( mmInt p_iCloudID, mmInt p_iStartPoint, mmInt p_iBlockCount )
{

	CS->LockAllDataForRead(); {

		CS->GetPoints(p_iCloudID, p_iStartPoint, p_iBlockCount, NULL, NULL, m_piStates, NULL, NULL, NULL, NULL);

		for (mmInt i = 0; i < p_iBlockCount; ++i)
			if (!(m_piStates[i] & 0x06)) m_piStates[i] ^= 0x01;

	} CS->UnlockAllDataFromRead();

	CS->LockAllDataForWrite(); {

		CS->SetPoints(p_iCloudID, p_iStartPoint, p_iBlockCount, NULL, m_piStates, NULL, NULL, NULL, NULL);

	} CS->UnlockAllDataFromWrite();
}
