#include "cmFindStudents.h"
#include "mmOperatingSystemCalls.h"
#include "math.h"
#include "math\mmMath.h"

static const
mmClouds::mmCloudOfPointsCalculationMethodI::sCalculationMethodParams cmEmptyMethodParams = {
	L"Find Students",
	L"da15a7e8-03d1-11e5-8418-1697f925ec7b",
	L"Detects students in given cloud of points",
	false // TODO: change to true if multi-threaded
};

mmClouds::cmFindStudents::cmFindStudents(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces) :
mmCalcMethod(p_psGlobalInterfaces, cmEmptyMethodParams)
{
	FUNC_SCOPE;
	m_bAlwaysConfirm = true;
	m_bAutoSelect = true;
	BindParam(m_rPlaneDeletionRadius, L"Promien detekcji plaszczyzn", Real, Input);
	BindParam(m_rPlaneDeletionMaxError, L"Max blad dopasowania plaszczyzny", Real, Input);
	BindParam(m_rSegmentationRadius, L"Promien segmentacji", Real, Input);
	BindParam(m_sCloudName, L"Nazwa chmury", CloudName);
}

void mmClouds::cmFindStudents::Calculate()
{
	FUNC_SCOPE;

	//=============================== sortowanie chmur ======================================
	sortCloudOfPoints();
	
	
	loadCloudOfPoints();

	//============================ usuwanie plaszczyzn =======================================
	clearCloudOfPointsFromLargeSurfaces();

	//============================= segmentacja ==============================================
	//segmentateCloudOfPoints();	//jako tako dziala

	
	//============================ rozpoznawanie twarzy po HSV ===============================
	//detectFacesByHSV();

	//============================== zapis wynikow ===========================================
	saveCloudOfPoints();
	//saveResultsToFile();

}


////////////////////////////////////////////////////////////////////////////////
/// TODO
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::sortCloudOfPoints()
{
	m_psCloudStructure->LockAllDataForWrite();	//lock

	m_psCloudStructure->SortXYZ();

	m_psCloudStructure->UnlockAllDataFromWrite();	//unlock
}



////////////////////////////////////////////////////////////////////////////////
/// loading cloud of points provided by user 
/// and initializing members for ongoing data processing
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::loadCloudOfPoints()
{
	m_psCloudStructure->LockAllDataForRead();	//lock
	
	m_iID = m_psCloudStructure->GetDirectionalCloudOfPointsID(m_sCloudName);
	m_iPointsCount = m_psCloudStructure->GetPointsCount(m_iID);
	m_iPointStart = 0;

	m_rGlobalXYZ = new mmReal[m_iPointsCount * 3];
	m_rNXNYNZ = new mmReal[m_iPointsCount * 3];
	m_iPointsState = new mmInt[m_iPointsCount];
	m_ucRGBA = new unsigned char[m_iPointsCount * 4];

	m_psCloudStructure->GetPoints(m_iID, m_iPointStart, m_iPointsCount, NULL, m_rGlobalXYZ, m_iPointsState, m_rNXNYNZ, NULL, m_ucRGBA, NULL);

	m_psCloudStructure->UnlockAllDataFromRead();	//unlock
}


////////////////////////////////////////////////////////////////////////////////
/// TODO
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::clearCloudOfPointsFromLargeSurfaces()
{
	m_psCloudStructure->LockAllDataForWrite();	//lock
	/*
	data layer
	*/
	mmInt v_iLayer = m_psCloudStructure->AddNewDataLayer(L"PlaneDetected", -1.0);
	mmReal *planes = new mmReal[m_iPointsCount];
	/*
	error variables
	*/
	mmReal *error = new mmReal(0.0);
	mmReal *maxError = new mmReal(0.0);
	mmReal *minError = new mmReal(1000.0);
	/*
	neighbourshoods*/
	std::vector<mmClouds::mmCloudsOfPointsStructureI::sPointID> v_vNeighbourPoints;		//wektor zawierajacy ID punktow z sasiedztwa
	mmInt v_iPointsInNeighbourhood = 0;		//ilosc punktow w sasiedztwie

	for (mmInt i = 0; i < m_iPointsCount; i++)
	{
		do
		{
			m_psCloudStructure->GetPointsInRadius(m_rGlobalXYZ[3 * i], m_rGlobalXYZ[3 * i + 1], m_rGlobalXYZ[3 * i + 2], m_rPlaneDeletionRadius, &v_vNeighbourPoints, &v_iPointsInNeighbourhood, m_iID);
			
			if (5 < v_iPointsInNeighbourhood && v_iPointsInNeighbourhood < 25)
			{
				std::vector<mmMath::sPoint3D> v_prGlobalXYZPointsInRadius2;
				for (mmInt j = 0; j < v_iPointsInNeighbourhood; j++) {
					mmMath::sPoint3D tempPoint3D;
					tempPoint3D.rX = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID)];
					tempPoint3D.rY = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID) + 1];
					tempPoint3D.rZ = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID) + 2];
					v_prGlobalXYZPointsInRadius2.push_back(tempPoint3D);
				}
				/*
				wyznaczenie plaszczyzny i przypisanie jej wektora normalnego
				*/
				mmMath::sPlane3D sp_AproxPlane;
				sp_AproxPlane = mmMath::CalcBestPlane3D(v_prGlobalXYZPointsInRadius2, error);

				if (*error < *minError)
					*minError = *error;

				if (*error > *maxError)
					*maxError = *error;

				//*acceptanceLevel = 0.001 + sqrt(0.001*v_iPointsInNeighbourhood);

				if (*error <= m_rPlaneDeletionMaxError)
				{
					planes[i] = 1.0;
					if (!(m_iPointsState[i] & mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected))
					{
						m_iPointsState[i] += mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected;
					};
				}
				else
				{
					planes[i] = 0.0;
				}
			}
			else if (v_iPointsInNeighbourhood < 5)
			{
				m_rPlaneDeletionRadius = 1.25*m_rPlaneDeletionRadius;
			}
			else if (v_iPointsInNeighbourhood > 25)
			{
				m_rPlaneDeletionRadius = 0.75*m_rPlaneDeletionRadius;
			}

		} while (v_iPointsInNeighbourhood < 5 || v_iPointsInNeighbourhood > 25);
		
		

		m_rProgress = (100.0*i / m_iPointsCount);
	}

	m_psCloudStructure->DeinitializeForMultithreadedCalculation();
	m_psCloudStructure->SetPointsAdditionalDataLayerCoord(m_iID, m_iPointStart, m_iPointsCount, v_iLayer, planes);
	
	m_psCloudStructure->UnlockAllDataFromWrite();	//unlock

	delete [] planes;

}

////////////////////////////////////////////////////////////////////////////////
/// TODO
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::segmentateCloudOfPoints()
{
	m_psCloudStructure->LockAllDataForWrite();
	{
		mmInt v_iLayer = m_psCloudStructure->AddNewDataLayer(L"Segment_number", -1.0);
		m_rGroupNumber = new mmReal[m_iPointsCount];
		mmInt * v_iCalculationState = new mmInt[m_iPointsCount];

		std::vector<mmClouds::mmCloudsOfPointsStructureI::sPointID> v_vNeighbourPoints;		//wektor zawierajacy ID punktow z sasiedztwa
		mmInt v_iPointsInNeighbourhood = 0;		//ilosc punktow w sasiedztwie

		for (mmInt i = 0; i < m_iPointsCount; i++)
		{
			m_rGroupNumber[i] = 0;
			v_iCalculationState[i] = 0;
		}

		mmInt v_igroupNumber = 0;

		//dla kazdego punktu
		for (mmInt i = 0; i < m_iPointsCount; i++)
		{
			if (m_rGroupNumber[i] == 0 && v_iCalculationState[i] == 0)
			{
				v_igroupNumber++;
				m_rGroupNumber[i] = v_igroupNumber;
				v_iCalculationState[i] = 1;

				m_psCloudStructure->GetPointsInRadius(m_rGlobalXYZ[3 * i], m_rGlobalXYZ[3 * i + 1], m_rGlobalXYZ[3 * i + 2], m_rSegmentationRadius, &v_vNeighbourPoints, &v_iPointsInNeighbourhood, m_iID);
				bool uncalculatedPointFromGroupFound = false;
				for (mmInt j = 0; j < v_iPointsInNeighbourhood; j++)
				{
					if (m_rGroupNumber[v_vNeighbourPoints[j].iPointID] == 0)
					{
						m_rGroupNumber[v_vNeighbourPoints[j].iPointID] = v_igroupNumber;
						uncalculatedPointFromGroupFound = true;
					}
				}
				
				while (uncalculatedPointFromGroupFound)
				{
					for (mmInt j = 0; j < m_iPointsCount; j++)
					{
						uncalculatedPointFromGroupFound = false;
						if (m_rGroupNumber[j] == v_igroupNumber && v_iCalculationState[j] == 0)
						{
							m_psCloudStructure->GetPointsInRadius(m_rGlobalXYZ[3 * j], m_rGlobalXYZ[3 * j + 1], m_rGlobalXYZ[3 * j + 2], m_rSegmentationRadius, &v_vNeighbourPoints, &v_iPointsInNeighbourhood, m_iID);

							for (mmInt k = 0; k < v_iPointsInNeighbourhood; k++)
							{
								if (m_rGroupNumber[v_vNeighbourPoints[k].iPointID] == 0)
								{
									m_rGroupNumber[v_vNeighbourPoints[k].iPointID] = v_igroupNumber;
									uncalculatedPointFromGroupFound = true;
								}
							}
							v_iCalculationState[j] == 1;
						}
					}
				}
			}
			m_rProgress = (100.0*i / m_iPointsCount);
		}
		m_psCloudStructure->DeinitializeForMultithreadedCalculation();
		m_psCloudStructure->SetPointsAdditionalDataLayerCoord(m_iID, m_iPointStart, m_iPointsCount, v_iLayer, m_rGroupNumber);
		delete []m_rGroupNumber;
		delete []v_iCalculationState;
	}

	m_psCloudStructure->UnlockAllDataFromWrite();
}

////////////////////////////////////////////////////////////////////////////////
/// TODO
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::detectFacesByHSV()
{
	m_psCloudStructure->LockAllDataForRead();

	m_psCloudStructure->UnlockAllDataFromRead();

}

////////////////////////////////////////////////////////////////////////////////
/// TODO
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::saveCloudOfPoints()
{
	m_psCloudStructure->DeinitializeForMultithreadedCalculation();
	m_psCloudStructure->LockAllDataForWrite();
	m_psCloudStructure->SetPoints(m_iID, m_iPointStart, m_iPointsCount, m_rGlobalXYZ, m_iPointsState, m_rNXNYNZ, NULL, NULL, NULL);
	m_psCloudStructure->UnlockAllDataFromWrite();
}

////////////////////////////////////////////////////////////////////////////////
/// TODO
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::saveResultsToFile()
{
	m_psCloudStructure->LockAllDataForRead();	//lock

	m_psCloudStructure->UnlockAllDataFromRead();	//unlock
}