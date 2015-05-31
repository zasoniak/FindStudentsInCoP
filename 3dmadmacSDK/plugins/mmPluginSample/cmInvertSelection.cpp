#include "cmInvertSelection.h"

static const
mmClouds::mmCloudOfPointsCalculationMethodI::sCalculationMethodParams cmInvertSelectionParams = {
	L"Sample: Invert point selection",
	L"{0A69F841-2B3A-452C-BCA4-0DD4F47115C5}",
	L"Finds complementary set of selected points",
};

mmClouds::cmInvertSelection::cmInvertSelection(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces) : 
	mmCalcMethod(p_psGlobalInterfaces, cmInvertSelectionParams)
{
	FUNC_SCOPE;
	BindParam(m_bVisibleCloudsOnly, L"Visible clouds only", Bool);
}

mmClouds::cmInvertSelection::~cmInvertSelection(void)
{
	FUNC_SCOPE;
}

void mmClouds::cmInvertSelection::Calculate() 
{
	FUNC_SCOPE;

	const mmInt v_iPointsPerPass(100000);

	mmInt v_iAllPointsCount(0);
	mmInt v_iPointsDone(0);
	mmInt v_iPointsCount(0);
	mmInt v_iPointStart(0);
	mmInt v_iPointEnd(0);

	// point state buffer and its memory handle
	mmInt *v_piBuffer;
	mmMemory::mmMemHandle v_sMemHandle;

	const mmInt v_iStateTest = mmCloudsOfPointsStructureI::mmCoP_deleted | mmCloudsOfPointsStructureI::mmCoP_selection_blocked;

	v_sMemHandle = m_psMemManager->AllocateMemory(v_iPointsPerPass*sizeof(*v_piBuffer));
	
	v_piBuffer = reinterpret_cast<mmInt*>(m_psMemManager->LockPointerForWriteFromHandle(v_sMemHandle));

	m_psCloudStructure->LockAllDataForRead(); {
		v_iAllPointsCount = m_psCloudStructure->GetPointsCount();
	} m_psCloudStructure->UnlockAllDataFromRead();

	auto v_vClouds = GetCloudIDs();

	// for all clouds
	for (auto ID = v_vClouds.begin(); ID != v_vClouds.end() && !m_bStopExecution; ++ID) {

		// initialize start/end indices
		v_iPointStart = v_iPointEnd = 0;

		m_psCloudStructure->LockAllDataForRead(); {
			v_iPointsCount = m_psCloudStructure->GetPointsCount(*ID);
		} m_psCloudStructure->UnlockAllDataFromRead();


		while (v_iPointStart < v_iPointsCount && !m_bStopExecution) {

			// update end point index
			if ((v_iPointEnd += v_iPointsPerPass) > v_iPointsCount) v_iPointEnd = v_iPointsCount;
			const mmInt v_iPassCount = v_iPointEnd - v_iPointStart;

			if (m_bStopExecution) break;

			// get state buffer
			m_psCloudStructure->LockAllDataForRead(); {
				m_psCloudStructure->GetPoints(*ID, v_iPointStart, v_iPassCount, NULL, NULL, v_piBuffer, NULL, NULL, NULL, NULL);

			} m_psCloudStructure->UnlockAllDataFromRead();

			register mmInt* v_piCurrent = v_piBuffer;
			for (mmInt* v_piEnd = v_piCurrent + v_iPassCount; v_piCurrent != v_piEnd; ++v_piCurrent) {
				if (m_bStopExecution) break;
				if (!(*v_piCurrent & v_iStateTest)) // if point is not deleted or blocked
					*v_piCurrent ^= mmCloudsOfPointsStructureI::mmCoP_selected; // flip selection bit
			}

			if (m_bStopExecution) break;

			// update point state from buffer
				m_psCloudStructure->LockAllDataForWrite(); {
					m_psCloudStructure->SetPoints(*ID, v_iPointStart, v_iPassCount, NULL, v_piBuffer, NULL, NULL, NULL, NULL);
			} m_psCloudStructure->UnlockAllDataFromWrite();

			// update start point index
			v_iPointStart = v_iPointEnd;

			// update progress
			v_iPointsDone += v_iPassCount;
			m_rProgress = 100.0*v_iPointsDone / v_iAllPointsCount;

		}

	}

	m_psMemManager->UnlockHandleFromWritePointer(v_sMemHandle);

	// free buffer memory
	m_psMemManager->FreeMemory(v_sMemHandle);
}