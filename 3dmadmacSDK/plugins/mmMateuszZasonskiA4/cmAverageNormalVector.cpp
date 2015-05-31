#include "mmAverageNormalVector.h"
#include "mmOperatingSystemCalls.h"
#include "math.h"
#include "math\mmMath.h"

static const
	mmClouds::mmCloudOfPointsCalculationMethodI::sCalculationMethodParams cmEmptyMethodParams = {
		L"Average Normal Vector",
		L"AverageNormalVector_PlaneAdjustment",
		L"Wyznaczenie sredniego wektora normalnego (przez dopasowanie plaszczyzny)",
		false // TODO: change to true if multi-threaded
};

mmClouds::cmAverageNormalVector::cmAverageNormalVector(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces) :
		mmCalcMethod(p_psGlobalInterfaces, cmEmptyMethodParams)
		{
			FUNC_SCOPE;
			m_bAlwaysConfirm = true;
			m_bAutoSelect = true;
			BindParam(m_sRadius, L"Promien usredniania", Real, Input);
			BindParam(m_sCloudName, L"Nazwa chmury", CloudName);
		}

void mmClouds::cmAverageNormalVector::Calculate()
		{
			FUNC_SCOPE;
			/*
			sortowanie chmury
			*/
			m_psCloudStructure->LockAllDataForWrite();
			{
				m_psCloudStructure->SortXYZ();
			} m_psCloudStructure->UnlockAllDataFromWrite();

			m_psCloudStructure->LockAllDataForRead();
			
				/*
				wczytujemy id chmury wskazanej przez uzytkownika
				*/
				m_iID = m_psCloudStructure->GetDirectionalCloudOfPointsID(m_sCloudName);	
				mmInt v_iPointsCount = m_psCloudStructure->GetPointsCount(m_iID);	//ilosc punktow w chmurze

				mmInt v_iPointStart = 0;	//punkt startowy
				mmReal *v_prGlobalXYZ = new mmReal[v_iPointsCount * 3];	//wektor zawierajacy wspolrzedne punktow w chmurze
				mmReal *v_prNXNYNZ = new mmReal[v_iPointsCount * 3];	//wektor zawierajacy wspolczynniki wektorow normalnych
				
				/*
				wczytujemy wspolrzedne punktow wybranej chmury
				*/
				m_psCloudStructure->GetPoints(m_iID, v_iPointStart, v_iPointsCount, NULL, v_prGlobalXYZ, NULL, v_prNXNYNZ, NULL, NULL, NULL);	

				std::vector<mmClouds::mmCloudsOfPointsStructureI::sPointID> v_vNeighbourPoints;		//wektor zawierajacy ID punktow z sasiedztwa
				mmInt v_iPointsInNeighbourhood = 0;		//ilosc punktow w sasiedztwie

				/*
				wyznaczenie wektora normalnego poprzez dopasowanie plaszczyzny do punktow z sasiedztwa
				iterujemy dla kazdego punktu w chmurze
				*/
				for (mmInt i = 0; i< v_iPointsCount; i++)
				{
					/*
					wyszukanie punktow w sasiedztwie
					*/
					m_psCloudStructure->GetPointsInRadius(v_prGlobalXYZ[3 * i], v_prGlobalXYZ[3 * i + 1], v_prGlobalXYZ[3 * i + 2], m_sRadius, &v_vNeighbourPoints, &v_iPointsInNeighbourhood, m_iID);
					
					/*
					przepisanie potrzebnych nam wspolrzednych punktow w sasiedztwie
					*/
					//mmReal *v_prGlobalXYZPointsInRadius = new mmReal[v_iPointsInNeighbourhood * 3];	//wspolrzedne punktow znalezionych w danym promieniu
					//for (mmInt j = 0; j < v_iPointsInNeighbourhood;j++) {
					//	v_prGlobalXYZPointsInRadius[3 * j] = v_prGlobalXYZ[3*((v_vNeighbourPoints[j]).iPointID)];
					//	v_prGlobalXYZPointsInRadius[3 * j + 1] = v_prGlobalXYZ[3*((v_vNeighbourPoints[j]).iPointID) + 1];
					//	v_prGlobalXYZPointsInRadius[3 * j + 2] = v_prGlobalXYZ[3*((v_vNeighbourPoints[j]).iPointID) + 2];
					//}
					
					std::vector<mmMath::sPoint3D> v_prGlobalXYZPointsInRadius2;
					for (mmInt j = 0; j < v_iPointsInNeighbourhood; j++) {
						mmMath::sPoint3D tempPoint3D;
						tempPoint3D.rX = v_prGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID)];
						tempPoint3D.rY = v_prGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID) + 1];
						tempPoint3D.rZ = v_prGlobalXYZ[3 * ((v_vNeighbourPoints[j]).iPointID) + 2];
						v_prGlobalXYZPointsInRadius2.push_back(tempPoint3D);
					}
					/*
					wyznaczenie plaszczyzny i przypisanie jej wektora normalnego
					*/

					mmMath::sPlane3D sp_AproxPlane;
					//sp_AproxPlane = mmMath::CalcBestPlane3D(v_prGlobalXYZPointsInRadius, v_iPointsInNeighbourhood, NULL, NULL);
					sp_AproxPlane = mmMath::CalcBestPlane3D(v_prGlobalXYZPointsInRadius2,NULL);
					v_prNXNYNZ[3 * i] = sp_AproxPlane.rA;
					v_prNXNYNZ[3 * i+1] = sp_AproxPlane.rB;
					v_prNXNYNZ[3 * i+2] = sp_AproxPlane.rC;
					

					/*
					GUI pokazanie postepu obliczen
					*/
					m_rProgress = (100.0*i / v_iPointsCount);
					
					//delete[] v_prGlobalXYZPointsInRadius;
				}

				m_psCloudStructure->UnlockAllDataFromRead();


				/*
				zapisanie danych wektorow normalnych do chmury
				*/
				m_psCloudStructure->DeinitializeForMultithreadedCalculation();
				m_psCloudStructure->LockAllDataForWrite();
				m_psCloudStructure->SetPoints(m_iID, v_iPointStart, v_iPointsCount, NULL, NULL, v_prNXNYNZ, NULL, NULL, NULL);
				m_psCloudStructure->UnlockAllDataFromWrite();

				delete[] v_prGlobalXYZ;
				delete[] v_prNXNYNZ;
			


			/*
			aktualizacja chmury punktow
			*/
			m_psCloudStructure->LockAllDataForRead();
			{
				m_psCloudStructure->UpdateCloudOfPoints();
			} m_psCloudStructure->UnlockAllDataFromRead();
		}


