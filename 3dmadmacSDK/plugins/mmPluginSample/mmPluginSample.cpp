#include <plugins/mmPluginCommon.h>

#include "cmInvertSelection.h"
#include "cmInvertSelectionEx.h"
#include "cmLoneliness.h"
#include "cmIOParameters.h"

// TODO: JK move global interfaces to mmPluginCommon GetGlobalInterfaces

mmClouds::mmCloudOfPointsCalculationMethodI* GetCalculationMethod( mmInt p_iCalculationMethodIndex, 
                                                                   mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces) 
{
	switch (p_iCalculationMethodIndex) {

	case __COUNTER__: return new cmInvertSelection(p_psGlobalInterfaces);

	case __COUNTER__: return new cmInvertSelectionEx(p_psGlobalInterfaces, true);
	case __COUNTER__: return new cmInvertSelectionEx(p_psGlobalInterfaces, false);

	case __COUNTER__: return new cmIOParameters(p_psGlobalInterfaces);
	case __COUNTER__: return new cmLoneliness(p_psGlobalInterfaces);

	default: return NULL;
	}
}