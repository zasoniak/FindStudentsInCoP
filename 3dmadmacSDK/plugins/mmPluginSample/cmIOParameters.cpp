#include "cmIOParameters.h"

static const
mmClouds::mmCloudOfPointsCalculationMethodI::sCalculationMethodParams cmIOParametersParams = {
	L"Sample: Input/Output Parameters",
	L"{33E17C3D-4AC7-43A2-B605-B97454E492A1}",
	L"Shows how various parameters get parsed"
};

mmString mmClouds::cmIOParameters::m_sPersistentTest = L"remember what you wrote...";

// persistent helper variables for demonstrating additional options
static bool g_bAutoSelectPersist = false;
static bool g_bVisibleCloudsOnlyPersist = false;

mmClouds::cmIOParameters::cmIOParameters(mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces) : 
                                       mmCalcMethod(p_psGlobalInterfaces, cmIOParametersParams)
{
	FUNC_SCOPE;

	// input parameters
	BindParam(m_bTest = true, L"m_bTest (true/false)", Bool);
	BindParam(m_iTest = 42, L"m_iTest (integer)", Int);
	BindParam(m_rTest = 3.14, L"m_rTest (floating point)", Real);
	BindParam(m_sTest = L"Some new name", L"m_sTest (string)", String);
	BindParam(m_sDLTest, L"m_sDLTest (data layer name)", LayerName);
	BindParam(m_sCloudTest, L"m_sCloudTest (cloud name)", CloudName);

	// persistent parameters
	BindParam(m_sPersistentTest, L"m_sPersistentTest (static variable)", String);

	// this won't be visible in the GUI
	BindParam(m_iTestOutput = m_iTest, L"m_iTestOutput (only as output)", Int, Output);

	// this however, will
	BindParam(m_iTestInOut = 7, L"m_iTestInOut (both input and output)", Int, InOut);

	// additional options
	BindParam(m_bAutoSelect = g_bAutoSelectPersist, L"Use m_bAutoSelect to choose first item in combo boxes", Bool);
	BindParam(m_bAlwaysConfirm, L"Use m_bAlwaysConfirm to show this window even when there are no input params", Bool);
	BindParam(m_bVisibleCloudsOnly = g_bVisibleCloudsOnlyPersist, L"Use m_bVisibleCloudsOnly to disable hidden clouds processing", Bool);

	// NOTE: binding local variables would be a mistake...
	mmString v_sLocalString;
	// BindParam(v_sLocalString, ...)
}

void mmClouds::cmIOParameters::Calculate() 
{
	FUNC_SCOPE;

	g_bAutoSelectPersist = m_bAutoSelect;
	g_bVisibleCloudsOnlyPersist = m_bVisibleCloudsOnly;

	m_iTestOutput = m_rTest*m_iTest;
}