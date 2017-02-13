//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: searchEnergy.cpp,v 1.42 2009/09/13 19:57:38 hao Exp $
//
//######################################################################

#include "searchEnergyFactory.h"
#include "contactEnergy.h"
#include "potentialQualityEnergy.h"
#include "autoGraspQualityEnergy.h"
#include "guidedPotentialQualityEnergy.h"
#include "guidedAutoGraspEnergy.h"
#include "strictAutoGraspEnergy.h"
#include "compliantEnergy.h"
#include "dynamicAutoGraspEnergy.h"


SearchEnergyFactory * SearchEnergyFactory::searchEnergyFactory = NULL;

void
SearchEnergyFactory::registerBuiltinCreators()
{
  REGISTER_SEARCH_ENERGY_CREATOR("CONTACT_ENERGY", ContactEnergy);
  REGISTER_SEARCH_ENERGY_CREATOR("POTENTIAL_QUALITY_ENERGY", PotentialQualityEnergy);
  REGISTER_SEARCH_ENERGY_CREATOR("AUTO_GRASP_QUALITY_ENERGY", AutoGraspQualityEnergy);
  REGISTER_SEARCH_ENERGY_CREATOR("GUIDED_POTENTIAL_QUALITY_ENERGY", GuidedPotentialQualityEnergy);
  REGISTER_SEARCH_ENERGY_CREATOR("GUIDED_AUTO_GRASP_QUALITY_ENERGY", GuidedAutoGraspQualityEnergy);
  REGISTER_SEARCH_ENERGY_CREATOR("STRICT_AUTO_GRASP_ENERGY", StrictAutoGraspEnergy);
  REGISTER_SEARCH_ENERGY_CREATOR("COMPLIANT_ENERGY", CompliantEnergy);
  REGISTER_SEARCH_ENERGY_CREATOR("DYNAMIC_AUTO_GRASP_ENERGY", DynamicAutoGraspEnergy);

}
