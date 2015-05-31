#include "cmLoneliness.h"
#include "mmOperatingSystemCalls.h"

static const
mmClouds::mmCloudOfPointsCalculationMethodI::sCalculationMethodParams cmLonelinessParams = {
	L"Sample: Create DL with nearest neighbor distance",
	L"{A6B26886-870D-492C-AF90-C07A81908DD6}",
	L"Creates new data layer storing distances from nearest neighbor",
	true
};

mmClouds::cmLoneliness::cmLoneliness(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces) : 
                                       mmCalcMethodEx(p_psGlobalInterfaces, cmLonelinessParams)
{
	FUNC_SCOPE;
	m_sDLName = L"Loneliness";
	BindParam(m_sDLName, L"New layer name", String, InOut);

	m_iPointsPerBlock = 1000;
}

void mmClouds::cmLoneliness::Calculate() 
{
	FUNC_SCOPE;

	mmInt v_iThreadID = mmOperatingSystem::GetCurrentThreadID();

	m_psBlockSync->Lock(); {
		m_mCoordBuffers[v_iThreadID] = new mmMath::sPoint3D[m_iPointsPerBlock];
		m_mDLBuffers[v_iThreadID] = new mmReal[m_iPointsPerBlock];
	} m_psBlockSync->Unlock();

	ForEachCloud(this);

	m_psBlockSync->Lock(); {
		delete [] m_mCoordBuffers[v_iThreadID];
		delete [] m_mDLBuffers[v_iThreadID];
	} m_psBlockSync->Unlock();

}

void mmClouds::cmLoneliness::ForEachBlock( mmInt p_iCloudID, mmInt p_iStartPoint, mmInt p_iBlockCount) {

	mmInt v_iThreadID = mmOperatingSystem::GetCurrentThreadID();

	mmMath::sPoint3D* p_sBlock = m_mCoordBuffers[v_iThreadID];
	mmReal* p_rOutBlock = m_mDLBuffers[v_iThreadID];


	CS->LockAllDataForRead(); {
		CS->GetPoints(p_iCloudID, p_iStartPoint,p_iBlockCount, (mmReal*) p_sBlock, NULL, NULL, NULL, NULL, NULL, NULL);
		mmReal v_iAvgDist = CS->GetAveragePointToPointDistance(p_iCloudID)*4;

		std::vector<mmCloudsOfPointsStructureI::sPointID> v_vNeighbors;
		v_vNeighbors.reserve(100);

		std::vector<mmMath::sPoint3D> v_vNeighborCoords;
		v_vNeighborCoords.reserve(100);


		for (int i = 0; i < p_iBlockCount; ++i) {

			mmInt v_iFound(0);
			mmReal mindist = v_iAvgDist;

			v_vNeighbors.clear();
			CS->GetPointsInRadius(p_sBlock[i].rX, p_sBlock[i].rY, p_sBlock[i].rZ, v_iAvgDist, &v_vNeighbors, &v_iFound);


			if (v_iFound > 1) {

				v_vNeighborCoords.resize(v_iFound);

				mmMath::sPoint3D* v_rNeighborCoords = &v_vNeighborCoords.front();
				CS->GetPoints(&v_vNeighbors, (mmReal*)v_rNeighborCoords, NULL, NULL, NULL, NULL, NULL, NULL);

				for (mmInt j = 0; j < v_iFound; ++j) {
					mmReal distance = mmMath::CalcPointToPointDistance3D(v_rNeighborCoords[j], p_sBlock[i]);
					if (distance != 0 && distance < mindist) mindist = distance;
				}
			}

			p_rOutBlock[i] = mindist;
		}

		v_vNeighbors.clear();
		
	} CS->UnlockAllDataFromRead();

	CS->LockAllDataForWrite(); {
		CS->SetPointsAdditionalDataLayerCoord(p_iCloudID, p_iStartPoint, p_iBlockCount, m_iDLID, p_rOutBlock);
	} CS->UnlockAllDataFromWrite();

}

void mmClouds::cmLoneliness::OnInitialize() {
	CS->LockAllDataForWrite(); {
		CS->SortXYZ();
		//CS->DeinitializeForMultithreadedCalculation();
		if (CS->GetCloudsCount())
			m_iDLID = CS->AddNewDataLayer(m_sDLName, -1);
	}; CS->UnlockAllDataFromWrite();
}

void mmClouds::cmLoneliness::OnFinish() {
	CS->LockAllDataForRead(); {
		//CS->InitializeForMultithreadedCalculation(3);
		CS->UpdateCloudOfPoints();
	}; CS->LockAllDataForRead();
}
