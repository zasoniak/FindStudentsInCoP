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
	sortCloudOfPoints(); //OK

	//============================ wczytanie chmury puntkow =================================
	loadCloudOfPoints(); //OK

	//============================ usuwanie plaszczyzn =======================================
	clearCloudOfPointsFromLargeSurfaces(); //OK
	clearCloudOfPointsFromSurfaceEdges();

	//============================= segmentacja ==============================================
	segmentateCloudOfPoints();	//OK

	//============================ rozpoznawanie  ===============================
	//detectFacesByHSV();

	detectHumansByGeometry();

	//============================== zapis wynikow ===========================================
	saveCloudOfPoints();	//OK
	//saveResultsToFile();


	//============================ czyszczenie pamieci ===============================
	delete []m_rGroupNumber;
}


////////////////////////////////////////////////////////////////////////////////
/// sorting cloud of points
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::sortCloudOfPoints()
{
	m_psCloudStructure->LockAllDataForWrite(); //lock

	m_psCloudStructure->SortXYZ();

	m_psCloudStructure->UnlockAllDataFromWrite(); //unlock
}


////////////////////////////////////////////////////////////////////////////////
/// loading cloud of points provided by user 
/// and initializing members for ongoing data processing
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::loadCloudOfPoints()
{
	m_psCloudStructure->LockAllDataForRead(); //lock

	m_iID = m_psCloudStructure->GetDirectionalCloudOfPointsID(m_sCloudName);
	m_iPointsCount = m_psCloudStructure->GetPointsCount(m_iID);
	m_iPointStart = 0;

	m_rGlobalXYZ = new mmReal[m_iPointsCount * 3];
	m_rNXNYNZ = new mmReal[m_iPointsCount * 3];
	m_iPointsState = new mmInt[m_iPointsCount];
	m_ucRGBA = new unsigned char[m_iPointsCount * 4];

	m_psCloudStructure->GetPoints(m_iID, m_iPointStart, m_iPointsCount, NULL, m_rGlobalXYZ, m_iPointsState, m_rNXNYNZ, NULL, m_ucRGBA, NULL);

	m_psCloudStructure->UnlockAllDataFromRead(); //unlock
}


////////////////////////////////////////////////////////////////////////////////
/// clear cloud of points from large surfaces
/// detectsp planes and marks point to be ommited
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::clearCloudOfPointsFromLargeSurfaces()
{
	m_psCloudStructure->LockAllDataForRead(); //lock

	m_bPlanesDetecion = new bool[m_iPointsCount];

	mmReal* error = new mmReal(0.0);

	std::vector<mmClouds::mmCloudsOfPointsStructureI::sPointID> v_vNeighbourPoints; //vector containing points' ID from neighbourhood
	mmInt v_iPointsInNeighbourhood = 0; //number of points in neighbourhood

	for (mmInt i = 0; i < m_iPointsCount; i++)
	{
		do
		{
			m_psCloudStructure->GetPointsInRadius(m_rGlobalXYZ[3 * i], m_rGlobalXYZ[3 * i + 1], m_rGlobalXYZ[3 * i + 2], m_rPlaneDeletionRadius, &v_vNeighbourPoints, &v_iPointsInNeighbourhood, m_iID);

			if (5 < v_iPointsInNeighbourhood && v_iPointsInNeighbourhood < 25)
			{
				std::vector<mmMath::sPoint3D> v_prGlobalXYZPointsInRadius2;
				for (mmInt j = 0; j < v_iPointsInNeighbourhood; j++)
				{
					mmMath::sPoint3D tempPoint3D;
					tempPoint3D.rX = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID)];
					tempPoint3D.rY = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID) + 1];
					tempPoint3D.rZ = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID) + 2];
					v_prGlobalXYZPointsInRadius2.push_back(tempPoint3D);
				}

				mmMath::sPlane3D sp_AproxPlane = mmMath::CalcBestPlane3D(v_prGlobalXYZPointsInRadius2, error);

				if (*error <= m_rPlaneDeletionMaxError) //if plane is fitted well enough mark point as a part of plane
				{
					/*if (!(m_iPointsState[i] &mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected))
						m_iPointsState[i] += mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected;*/
					m_bPlanesDetecion[i] = true;
				}
				else
					m_bPlanesDetecion[i] = false;
			}
			else if (v_iPointsInNeighbourhood < 5) //optimize radius for reliable plane calculation
				m_rPlaneDeletionRadius = 1.25 * m_rPlaneDeletionRadius;
			else if (v_iPointsInNeighbourhood > 25) //optimize radius for reliable plane calculation
				m_rPlaneDeletionRadius = 0.75 * m_rPlaneDeletionRadius;
		}
		while (v_iPointsInNeighbourhood < 5 || v_iPointsInNeighbourhood > 25);

		m_rProgress = (100.0 * i / (5 * m_iPointsCount)); //it's just a part of algorithm
	}
	m_psCloudStructure->UnlockAllDataFromRead(); //unlock
}


////////////////////////////////////////////////////////////////////////////////
/// clear cloud of points from point on surfaces egde with similar color
////////////////////////////////////////////////////////////////////////////////
//void mmClouds::cmFindStudents::clearCloudOfPointsFromSurfaceEdges()
//{
//	m_psCloudStructure->LockAllDataForRead();
//	std::vector<mmClouds::mmCloudsOfPointsStructureI::sPointID> v_vNeighbourPoints; //wektor zawierajacy ID punktow z sasiedztwa
//	mmInt v_iPointsInNeighbourhood = 0; //ilosc punktow w sasiedztwie
//	double v_R, v_G, v_B;
//	double v_RtoCheck, v_GtoCheck, v_BtoCheck;
//
//	mmReal *h = new mmReal();
//	mmReal *s = new mmReal();
//	mmReal *v = new mmReal();
//
//	mmReal *htoCheck = new mmReal();
//	mmReal *stoCheck = new mmReal();
//	mmReal *vtoCheck = new mmReal();
//
//	bool* v_planes = new bool[m_iPointsCount];
//	for (mmInt i = 0; i < m_iPointsCount; i++)
//	{
//		v_planes[i] = false;
//	}
//	for (mmInt i = 0; i < m_iPointsCount; i++)
//	{
//		if (m_bPlanesDetecion[i] == true)
//		{
//			m_psCloudStructure->GetPointsInRadius(m_rGlobalXYZ[3 * i], m_rGlobalXYZ[3 * i + 1], m_rGlobalXYZ[3 * i + 2], m_rPlaneDeletionRadius/3, &v_vNeighbourPoints, &v_iPointsInNeighbourhood, m_iID);
//			
//		
//			v_R = ((unsigned int) m_ucRGBA[4 * i]) / 255.0; //R value 
//			v_G = ((unsigned int) m_ucRGBA[4 * i + 1]) / 255.0; //G value
//			v_B = ((unsigned int) m_ucRGBA[4 * i + 2]) / 255.0; //B value
//			RGBtoHSV(v_R, v_G, v_B, h, s, v);
//
//
//			for (mmInt j = 0; j < v_iPointsInNeighbourhood; j++)
//			{
//				//sprawdzic kolor
//
//	
//				v_RtoCheck = ((unsigned int) m_ucRGBA[4 * v_vNeighbourPoints[j].iPointID]) / 255.0; //R value 
//				v_GtoCheck = ((unsigned int) m_ucRGBA[4 * v_vNeighbourPoints[j].iPointID + 1]) / 255.0; //G value
//				v_BtoCheck = ((unsigned int) m_ucRGBA[4 * v_vNeighbourPoints[j].iPointID + 2]) / 255.0; //B value
//				RGBtoHSV(v_RtoCheck, v_GtoCheck, v_BtoCheck, htoCheck, stoCheck, vtoCheck);
//
//				if (*htoCheck >= 0.8*(*h) && *htoCheck <= 1.2*(*h) &&
//					*stoCheck >= 0.8*(*s) && *stoCheck <= 1.2*(*s) &&
//					*vtoCheck >= 0.8*(*v) && *vtoCheck <= 1.2*(*v))
//				{
//					v_planes[v_vNeighbourPoints[j].iPointID] = true;
//				}
//			}
//		}
//	}
//
//	for (mmInt i = 0; i < m_iPointsCount; i++)
//	{
//		m_bPlanesDetecion[i] = m_bPlanesDetecion[i] || v_planes[i];
//	}
//	//for (mmInt i = 0; i < m_iPointsCount; i++)
//	//{
//	//	if (m_bPlanesDetecion[i]==true && !(m_iPointsState[i] &mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected))
//	//	m_iPointsState[i] += mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected;
//	//}
//	m_psCloudStructure->UnlockAllDataFromRead();
//}

void mmClouds::cmFindStudents::clearCloudOfPointsFromSurfaceEdges()	//wersja z rzutowaniem na plaszczyzny
{
	m_psCloudStructure->LockAllDataForRead();
	std::vector<mmClouds::mmCloudsOfPointsStructureI::sPointID> v_vNeighbourPoints; //wektor zawierajacy ID punktow z sasiedztwa
	mmInt v_iPointsInNeighbourhood = 0; //ilosc punktow w sasiedztwie


	bool* v_planes = new bool[m_iPointsCount];
	for (mmInt i = 0; i < m_iPointsCount; i++)
	{
		v_planes[i] = false;
	}

	mmReal* error = new mmReal(0.0);

	for (mmInt i = 0; i < m_iPointsCount; i++)
	{
		if (m_bPlanesDetecion[i] == true)
		{
			m_psCloudStructure->GetPointsInRadius(m_rGlobalXYZ[3 * i], m_rGlobalXYZ[3 * i + 1], m_rGlobalXYZ[3 * i + 2], m_rPlaneDeletionRadius, &v_vNeighbourPoints, &v_iPointsInNeighbourhood, m_iID);

			std::vector<mmMath::sPoint3D> v_prGlobalXYZPointsInRadius2;
			for (mmInt j = 0; j < v_iPointsInNeighbourhood; j++)
			{
				if (m_bPlanesDetecion[(v_vNeighbourPoints[j]).iPointID] == true)
				{
					mmMath::sPoint3D tempPoint3D;
					tempPoint3D.rX = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID)];
					tempPoint3D.rY = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID) + 1];
					tempPoint3D.rZ = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID) + 2];
					v_prGlobalXYZPointsInRadius2.push_back(tempPoint3D);
				}
			}
			mmMath::sPlane3D sp_AproxPlane = mmMath::CalcBestPlane3D(v_prGlobalXYZPointsInRadius2, error);


			for (mmInt j = 0; j < v_iPointsInNeighbourhood; j++)
			{
				if (m_bPlanesDetecion[(v_vNeighbourPoints[j]).iPointID] == false)
				{
					mmMath::sPoint3D tempPoint3D;
					tempPoint3D.rX = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID)];
					tempPoint3D.rY = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID) + 1];
					tempPoint3D.rZ = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID) + 2];
					mmReal distance = CalcPointToPlaneDistance3D(tempPoint3D, sp_AproxPlane, false);
					if (distance < 2)
						v_planes[(v_vNeighbourPoints[j]).iPointID] = true;
				}
			}
		}
	}

	for (mmInt i = 0; i < m_iPointsCount; i++)
	{
		m_bPlanesDetecion[i] = m_bPlanesDetecion[i] || v_planes[i];
	}
	/*for (mmInt i = 0; i < m_iPointsCount; i++)
	{
		if (m_bPlanesDetecion[i]==true && !(m_iPointsState[i] &mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected))
		m_iPointsState[i] += mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected;
	}*/
	m_psCloudStructure->UnlockAllDataFromRead();
}


////////////////////////////////////////////////////////////////////////////////
/// cloud of points segmentation using Hausdorf distance
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::segmentateCloudOfPoints()
{
	m_psCloudStructure->LockAllDataForWrite();
	m_psCloudStructure->DeinitializeForMultithreadedCalculation();
	mmInt v_iLayer = m_psCloudStructure->AddNewDataLayer(L"SegmentNumber", -1.0);
	m_rGroupNumber = new mmReal[m_iPointsCount];
	m_iGroupMembersCount = new mmInt[m_iPointsCount + 1];
	mmInt* v_iCalculationState = new mmInt[m_iPointsCount];

	std::vector<mmClouds::mmCloudsOfPointsStructureI::sPointID> v_vNeighbourPoints; //wektor zawierajacy ID punktow z sasiedztwa
	mmInt v_iPointsInNeighbourhood = 0; //ilosc punktow w sasiedztwie

	for (mmInt i = 0; i < m_iPointsCount; i++)
	{
		m_rGroupNumber[i] = 0;
		if (m_bPlanesDetecion[i] == true) //wyrzuca wyszukane plaszczyzny z algorytmu
			v_iCalculationState[i] = 1;
		else
			v_iCalculationState[i] = 0;
	}
	
	m_igroupNumber = 0;

	for (mmInt i = 0; i < m_iPointsCount; i++)
	{
		if (m_rGroupNumber[i] == 0 && v_iCalculationState[i] == 0)
		{
			m_igroupNumber++;
			m_iGroupMembersCount[m_igroupNumber] = 1;
			m_rGroupNumber[i] = m_igroupNumber;
			v_iCalculationState[i] = 1;

			m_psCloudStructure->GetPointsInRadius(m_rGlobalXYZ[3 * i], m_rGlobalXYZ[3 * i + 1], m_rGlobalXYZ[3 * i + 2], m_rSegmentationRadius, &v_vNeighbourPoints, &v_iPointsInNeighbourhood, m_iID);
			bool uncalculatedPointFromGroupFound = false;
			for (mmInt j = 0; j < v_iPointsInNeighbourhood; j++)
			{
				if (m_rGroupNumber[v_vNeighbourPoints[j].iPointID] == 0)
				{
					m_rGroupNumber[v_vNeighbourPoints[j].iPointID] = m_igroupNumber;
					m_iGroupMembersCount[m_igroupNumber]++;
					uncalculatedPointFromGroupFound = true;
				}
			}

			while (uncalculatedPointFromGroupFound)
			{
				for (mmInt j = 0; j < m_iPointsCount; j++)
				{
					uncalculatedPointFromGroupFound = false;
					if (m_rGroupNumber[j] == m_igroupNumber && v_iCalculationState[j] == 0)
					{
						m_psCloudStructure->GetPointsInRadius(m_rGlobalXYZ[3 * j], m_rGlobalXYZ[3 * j + 1], m_rGlobalXYZ[3 * j + 2], m_rSegmentationRadius, &v_vNeighbourPoints, &v_iPointsInNeighbourhood, m_iID);

						for (mmInt k = 0; k < v_iPointsInNeighbourhood; k++)
						{
							if (m_rGroupNumber[v_vNeighbourPoints[k].iPointID] == 0)
							{
								m_rGroupNumber[v_vNeighbourPoints[k].iPointID] = m_igroupNumber;
								m_iGroupMembersCount[m_igroupNumber]++;
								uncalculatedPointFromGroupFound = true;
							}
						}
						v_iCalculationState[j] = 1;
					}
				}
			}
		}

		m_rProgress = (100.0 * (m_iPointsCount + 4 * i) / (5 * m_iPointsCount));
	}

	mergeSegments();

	m_psCloudStructure->DeinitializeForMultithreadedCalculation();
	m_psCloudStructure->SetPointsAdditionalDataLayerCoord(m_iID, m_iPointStart, m_iPointsCount, v_iLayer, m_rGroupNumber);


	delete []v_iCalculationState;

	m_psCloudStructure->UnlockAllDataFromWrite();
}

void mmClouds::cmFindStudents::mergeSegments()
{
	std::vector<mmClouds::mmCloudsOfPointsStructureI::sPointID> v_vNeighbourPoints; //wektor zawierajacy ID punktow z sasiedztwa
	mmInt v_iPointsInNeighbourhood = 0; //ilosc punktow w sasiedztwie

	std::map < int, int >  v_iiGroupChangeLookup;

	for (mmInt i = m_iPointsCount-1; i >=0; i--)
	{
		if (m_rGroupNumber[i] != 0)
		{
			if (v_iiGroupChangeLookup.find(static_cast<int>(m_rGroupNumber[i])) != v_iiGroupChangeLookup.end())	//jesli juz byla podmiana tej grupy
				m_rGroupNumber[i] = v_iiGroupChangeLookup.find(static_cast<int>(m_rGroupNumber[i]))->second;

			m_psCloudStructure->GetPointsInRadius(m_rGlobalXYZ[3 * i], m_rGlobalXYZ[3 * i + 1], m_rGlobalXYZ[3 * i + 2], m_rSegmentationRadius, &v_vNeighbourPoints, &v_iPointsInNeighbourhood, m_iID);
			for (mmInt j = 0; j < v_iPointsInNeighbourhood; j++)
			{
				if (m_rGroupNumber[v_vNeighbourPoints[j].iPointID] != m_rGroupNumber[i] && m_rGroupNumber[v_vNeighbourPoints[j].iPointID] != 0)	//jesli sasiad innego segmentu przypisujemy obecny
				{
					if (v_iiGroupChangeLookup.find(static_cast<int>(m_rGroupNumber[i])) == v_iiGroupChangeLookup.end())	//zapamietujemy podmiane
					{
						v_iiGroupChangeLookup.insert(std::make_pair(static_cast<int>(m_rGroupNumber[v_vNeighbourPoints[j].iPointID]), m_rGroupNumber[i]));
					}
					m_rGroupNumber[v_vNeighbourPoints[j].iPointID] = m_rGroupNumber[i];

				}
			}
		} else 
		{
			if (v_iiGroupChangeLookup.find(static_cast<int>(m_rGroupNumber[i])) != v_iiGroupChangeLookup.end())	//jesli juz byla podmiana tej grupy
				m_rGroupNumber[i] = v_iiGroupChangeLookup.find(static_cast<int>(m_rGroupNumber[i]))->second;

			m_psCloudStructure->GetPointsInRadius(m_rGlobalXYZ[3 * i], m_rGlobalXYZ[3 * i + 1], m_rGlobalXYZ[3 * i + 2], m_rSegmentationRadius/2, &v_vNeighbourPoints, &v_iPointsInNeighbourhood, m_iID);
			for (mmInt j = 0; j < v_iPointsInNeighbourhood; j++)
			{
				if (m_rGroupNumber[v_vNeighbourPoints[j].iPointID] != m_rGroupNumber[i] && m_rGroupNumber[v_vNeighbourPoints[j].iPointID] != 0)	//jesli sasiad innego segmentu przypisujemy obecny
				{
					if (v_iiGroupChangeLookup.find(static_cast<int>(m_rGroupNumber[i])) == v_iiGroupChangeLookup.end())	//zapamietujemy podmiane
					{
						v_iiGroupChangeLookup.insert(std::make_pair(static_cast<int>(m_rGroupNumber[v_vNeighbourPoints[j].iPointID]), m_rGroupNumber[i]));
					}
					m_rGroupNumber[v_vNeighbourPoints[j].iPointID] = m_rGroupNumber[i];

				}
			}
		}
		m_rProgress = (100.0 * (m_iPointsCount + 4 * i) / (5 * m_iPointsCount));
	}

	int v_igroupCount = 1;

	v_iiGroupChangeLookup.clear();


	for (mmInt i = 0; i < m_iPointsCount; i++)	//minimalizes segments number
	{
		if (m_rGroupNumber[i] != 0 && v_iiGroupChangeLookup.find(static_cast<int>(m_rGroupNumber[i])) == v_iiGroupChangeLookup.end())
		{

			v_iiGroupChangeLookup.insert(std::make_pair(static_cast<int>(m_rGroupNumber[i]), v_igroupCount));
			v_igroupCount++;
		}
	}
	for (mmInt i = 0; i < m_iPointsCount; i++)	//set new segments to the points
	{
		if (v_iiGroupChangeLookup.find(static_cast<int>(m_rGroupNumber[i])) != v_iiGroupChangeLookup.end())
		{
			m_rGroupNumber[i] = (v_iiGroupChangeLookup.find(static_cast<int>(m_rGroupNumber[i])))->second;
		}
	}



}


////////////////////////////////////////////////////////////////////////////////
/// humans detection by faces hsv
/// finds points with hsv within specified range and marks whole group as human
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::detectFacesByHSV()
{
	m_psCloudStructure->LockAllDataForWrite();

	m_rGroupsMarkedAsHuman = new mmReal[m_igroupNumber + 1];
	int humansFound = 0;
	mmReal *h = new mmReal();
	mmReal *s = new mmReal();
	mmReal *v = new mmReal();

	for (mmInt i = 0; i < m_iPointsCount;i++)
	{
		if (m_rGroupNumber[i]!=0)
		{
			double v_R, v_G, v_B;
			v_R = ((unsigned int) m_ucRGBA[4 * i]) / 255.0; //R value 
			v_G = ((unsigned int) m_ucRGBA[4 * i + 1]) / 255.0; //G value
			v_B = ((unsigned int) m_ucRGBA[4 * i + 2]) / 255.0; //B value

			RGBtoHSV(v_R, v_G, v_B, h, s, v);

			if (25 < *h && *h < 70 &&
				0.4< *v && *v < 0.8 &&
				0.1 <*s && *s <0.4)
			{
				m_rGroupsMarkedAsHuman[static_cast<int>(m_rGroupNumber[i])] = 1;
				humansFound++;
			}
			else
			{
				m_rGroupsMarkedAsHuman[static_cast<int>(m_rGroupNumber[i])] = 0;
			}
		}
	}

	for (mmInt i = 0; i < m_iPointsCount; i++)
	{
		if (m_rGroupsMarkedAsHuman[static_cast<int>(m_rGroupNumber[i])] ==1)
		{
			if (!(m_iPointsState[i] &mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected))
			m_iPointsState[i] += mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected;
		}

	}

	m_psCloudStructure->UnlockAllDataFromWrite();
}


////////////////////////////////////////////////////////////////////////////////
/// human detection by geometry
/// finds places with irregular geometry and marks whole group as human
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::detectHumansByGeometry()
{
	m_psCloudStructure->LockAllDataForRead(); //lock
	m_rGroupsMarkedAsHuman = new mmReal[m_igroupNumber + 1];

	mmReal error =  mmReal(0.0);
	mmReal minError =  mmReal(10000.0);
	mmReal maxError =  mmReal(-10.0);

	std::vector<mmClouds::mmCloudsOfPointsStructureI::sPointID> v_vNeighbourPoints; //vector containing points' ID from neighbourhood
	mmInt v_iPointsInNeighbourhood = 0; //number of points in neighbourhood

	for (mmInt i = 0; i < m_iPointsCount; i++)
	{
		do
		{
			m_psCloudStructure->GetPointsInRadius(m_rGlobalXYZ[3 * i], m_rGlobalXYZ[3 * i + 1], m_rGlobalXYZ[3 * i + 2], m_rPlaneDeletionRadius, &v_vNeighbourPoints, &v_iPointsInNeighbourhood, m_iID);

			if (5 < v_iPointsInNeighbourhood && v_iPointsInNeighbourhood < 25)
			{
				std::vector<mmMath::sPoint3D> v_prGlobalXYZPointsInRadius2;
				for (mmInt j = 0; j < v_iPointsInNeighbourhood; j++)
				{
					mmMath::sPoint3D tempPoint3D;
					tempPoint3D.rX = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID)];
					tempPoint3D.rY = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID) + 1];
					tempPoint3D.rZ = m_rGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID) + 2];
					v_prGlobalXYZPointsInRadius2.push_back(tempPoint3D);
				}

				mmMath::sPlane3D sp_AproxPlane = mmMath::CalcBestPlane3D(v_prGlobalXYZPointsInRadius2, &error);

				if (error>4 && error<5) //if plane is fitted well enough mark point as a part of plane
				{
					m_rGroupsMarkedAsHuman[static_cast<int>(m_rGroupNumber[i])] = 1;
				}
				if (error < minError)
					minError = error;
				if (error > maxError)
					maxError = error;
			}
			else if (v_iPointsInNeighbourhood < 5) //optimize radius for reliable plane calculation
				m_rPlaneDeletionRadius = 1.25 * m_rPlaneDeletionRadius;
			else if (v_iPointsInNeighbourhood > 25) //optimize radius for reliable plane calculation
				m_rPlaneDeletionRadius = 0.75 * m_rPlaneDeletionRadius;
		} while (v_iPointsInNeighbourhood < 5 || v_iPointsInNeighbourhood > 25);

		m_rProgress = (100.0 * i / (5 * m_iPointsCount)); //it's just a part of algorithm
	}


	for (mmInt i = 0; i < m_iPointsCount; i++)
	{
		if (m_rGroupsMarkedAsHuman[static_cast<int>(m_rGroupNumber[i])] == 1 && m_rGroupNumber[i]!=0)
		{
			if (!(m_iPointsState[i] & mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected))
				m_iPointsState[i] += mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected;
		}

	}

	m_psCloudStructure->UnlockAllDataFromRead(); //unlock
}


////////////////////////////////////////////////////////////////////////////////
/// save cloud of points
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::saveCloudOfPoints()
{
	m_psCloudStructure->DeinitializeForMultithreadedCalculation();
	m_psCloudStructure->LockAllDataForWrite();
	m_psCloudStructure->SetPoints(m_iID, m_iPointStart, m_iPointsCount, m_rGlobalXYZ, m_iPointsState, m_rNXNYNZ, NULL, NULL, NULL);
	m_psCloudStructure->UnlockAllDataFromWrite();
}

////////////////////////////////////////////////////////////////////////////////
/// saves results to file
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::saveResultsToFile()
{
	m_psCloudStructure->LockAllDataForRead(); //lock

	m_psCloudStructure->UnlockAllDataFromRead(); //unlock
}



////////////////////////////////////////////////////////////////////////////////
/// calculates HSV from RGB values
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmFindStudents::RGBtoHSV(mmReal r, mmReal g, mmReal b, mmReal *h, mmReal *s, mmReal *v)
{
	mmReal minimum, maximum, delta;
	minimum = r < g ? r : g;
	minimum = minimum  < b ? minimum : b;

	maximum = r > g ? r : g;
	maximum = maximum  > b ? maximum : b;
	*v = maximum;				// v
	delta = maximum - minimum;
	if (maximum != 0)
		*s = delta / maximum;		// s
	else {
		// r = g = b = 0		// s = 0, v is undefined
		*s = 0;
		*h = -1;
		return;
	}
	if (r == maximum)
		*h = (g - b) / delta;		// between yellow & magenta
	else if (g == maximum)
		*h = 2 + (b - r) / delta;	// between cyan & yellow
	else
		*h = 4 + (r - g) / delta;	// between magenta & cyan
	*h *= 60;				// degrees
	if (*h < 0)
		*h += 360;
}