#include <plugins/mmPluginCommon.h>

// TODO: add calculation method headers
#include "cmFindStudents.h"

mmClouds::mmCloudOfPointsCalculationMethodI* GetCalculationMethod( mmInt p_iCalculationMethodIndex, 
                                                                   mmInterfacesStorage::mmGlobalInterfacesStorage* p_psGlobalInterfaces) 
{
	switch (p_iCalculationMethodIndex) {
		// TODO: add calculation methods
	case __COUNTER__: return new cmFindStudents(p_psGlobalInterfaces);


	default: return NULL;
	}
}