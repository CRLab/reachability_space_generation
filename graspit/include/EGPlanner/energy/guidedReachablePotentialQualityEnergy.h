#ifndef _guidedreachablepotentialenergy_h_
#define _guidedreachablepotentialenergy_h_

#include "EGPlanner/energy/searchEnergy.h"

class GuidedReachablePotentialQualityEnergy: public SearchEnergy
{
  public:
    double energy() const;

  protected:
    double potentialQualityEnergy() const;
    double contactEnergy() const;
    double potentialQualityScalingFunction(double dist, double cosTheta)const ;


};


#endif
