#include "cmEmptyMethod.h"

static const
mmClouds::mmCloudOfPointsCalculationMethodI::sCalculationMethodParams cmEmptyMethodParams = {
	L"Sample: Empty method",
	L"TODO: insert GUID here",
	L"TODO: provide method description",
	false // TODO: change to true if multi-threaded
};

mmClouds::cmEmptyMethod::cmEmptyMethod(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces) : 
                                       mmCalcMethod(p_psGlobalInterfaces, cmEmptyMethodParams)
{
	FUNC_SCOPE;
	// TODO: initialize/bind parameters
}

void mmClouds::cmEmptyMethod::Calculate() 
{
	FUNC_SCOPE;
	// TODO: implement some 3DIP algorithm
}