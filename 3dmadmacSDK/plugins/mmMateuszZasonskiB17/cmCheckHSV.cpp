#include "cmCheckHSV.h"
#include "mmOperatingSystemCalls.h"
#include "math.h"
#include "math\mmMath.h"

static const
mmClouds::mmCloudOfPointsCalculationMethodI::sCalculationMethodParams cmEmptyMethodParams = {
	L"Check HSV in selection",
	L"da11a7e8-01d1-1ce5-8418-16979145ec7b",
	L"Calculates HSV stats in selected are",
	false // TODO: change to true if multi-threaded
};

mmClouds::cmCheckHSV::cmCheckHSV(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces) :
mmCalcMethod(p_psGlobalInterfaces, cmEmptyMethodParams)
{
	FUNC_SCOPE;
	m_bAlwaysConfirm = true;
	m_bAutoSelect = true;
	BindParam(m_sCloudName, L"Nazwa chmury", CloudName);
}

void mmClouds::cmCheckHSV::Calculate()
{
	FUNC_SCOPE;

	//=============================== sortowanie chmur ======================================
	sortCloudOfPoints(); //OK

	//============================ wczytanie chmury puntkow =================================
	loadCloudOfPoints(); //OK

	//============================ rozpoznawanie twarzy po HSV ===============================
	checkHSV();
	checkSphere();


	//============================ czyszczenie pamieci ===============================
}


////////////////////////////////////////////////////////////////////////////////
/// sorting cloud of points
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmCheckHSV::sortCloudOfPoints()
{
	m_psCloudStructure->LockAllDataForWrite(); //lock

	m_psCloudStructure->SortXYZ();

	m_psCloudStructure->UnlockAllDataFromWrite(); //unlock
}


////////////////////////////////////////////////////////////////////////////////
/// loading cloud of points provided by user 
/// and initializing members for ongoing data processing
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmCheckHSV::loadCloudOfPoints()
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
/// TODO
////////////////////////////////////////////////////////////////////////////////
void mmClouds::cmCheckHSV::checkHSV()
{
	m_psCloudStructure->LockAllDataForWrite();

	int humansFound = 0;
	mmReal *h_max = new mmReal(-10);
	mmReal *s_max = new mmReal(-10);
	mmReal *v_max = new mmReal(-10);

	mmReal *h_min = new mmReal(100);
	mmReal *s_min = new mmReal(100);
	mmReal *v_min = new mmReal(100);

	mmReal *h_avg = new mmReal(0.0);
	mmReal *s_avg = new mmReal(0.0);
	mmReal *v_avg = new mmReal(0.0);

	mmReal *h = new mmReal();
	mmReal *s = new mmReal();
	mmReal *v = new mmReal();
	mmInt counter = 0;

	for (mmInt i = 0; i < m_iPointsCount; i++)
	{
		if ((m_iPointsState[i] & mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected))
		{
			counter++;
			double v_R, v_G, v_B;
			v_R = ((unsigned int) m_ucRGBA[4 * i]) / 255.0; //R value 
			v_G = ((unsigned int) m_ucRGBA[4 * i + 1]) / 255.0; //G value
			v_B = ((unsigned int) m_ucRGBA[4 * i + 2]) / 255.0; //B value

			if (v_R != 0 && v_G != 0 && v_B != 0)
			{
				RGBtoHSV(v_R, v_G, v_B, h, s, v);
				if (*h < *h_min)
					*h_min = *h;
				if (*s < *s_min)
					*s_min = *s;
				if (*v < *v_min)
					*v_min = *v;


				if (*h > *h_max)
					*h_max = *h;
				if (*s > *s_max)
					*s_max = *s;
				if (*v > *v_max)
					*v_max = *v;


				*h_avg += *h;
				*s_avg += *s;
				*v_avg += *v;
			}
		}
	}
	*h_avg /= counter;
	*s_avg /= counter;
	*v_avg /= counter;

	m_psCloudStructure->UnlockAllDataFromWrite();
}


void mmClouds::cmCheckHSV::checkSphere()
{
	std::vector<mmMath::sPoint3D>* p_pvInPoints = new std::vector<mmMath::sPoint3D>();
	mmInt maxSphereRadius = 150;
	mmInt minSphereRadius = 50;
	float checksum;
		for (mmInt i = 0; i < m_iPointsCount; i++)
		{
			if ((m_iPointsState[i] & mmClouds::mmCloudsOfPointsStructureI::mmCoP_selected))
			{
				mmMath::sPoint3D point;
				point.rX = m_rGlobalXYZ[3 * i];
				point.rY = m_rGlobalXYZ[3 * i + 1];
				point.rZ = m_rGlobalXYZ[3 * i + 2];
				p_pvInPoints->push_back(point);
			}
		}

		mmMath::sSphere3D sphere = mmMath::CalcSphere3DFormulaByMinRMS(p_pvInPoints);

		if (sphere.rRadius > minSphereRadius && sphere.rRadius < maxSphereRadius)
		{
			for (std::vector<mmMath::sPoint3D>::iterator PtIter = p_pvInPoints->begin(); PtIter != p_pvInPoints->end(); ++PtIter)
			{
				checksum = (PtIter->rX - sphere.rX0)*(PtIter->rX - sphere.rX0) + (PtIter->rY - sphere.rY0)*(PtIter->rY - sphere.rY0) + (PtIter->rZ - sphere.rZ0)*(PtIter->rZ - sphere.rZ0) - sphere.rRadius*sphere.rRadius;

			}
		}
		p_pvInPoints->clear();
}



void mmClouds::cmCheckHSV::RGBtoHSV(mmReal r, mmReal g, mmReal b, mmReal *h, mmReal *s, mmReal *v)
{
	mmReal minimum, maximum, delta;
	minimum = r < g ? r : g;
	minimum = minimum  < b ? minimum : b;

	maximum = r > g ? r : g;
	maximum = maximum  > b ? maximum : b;
	*v = maximum;				// v
	delta = maximum - minimum;
	if (delta == 0)
		delta = 1;
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