#ifndef _guidedreachablepotentialenergy_h_
#define _guidedreachablepotentialenergy_h_

#include "EGPlanner/energy/searchEnergy.h"
#include "EGPlanner/energy/reachableEnergyUtils.h"

class GuidedReachablePotentialQualityEnergy: public SearchEnergy
{
  public:
    double energy() const;
    GuidedReachablePotentialQualityEnergy();

  protected:
    double potentialQualityEnergy() const;
    double contactEnergy() const;
    double potentialQualityScalingFunction(double dist, double cosTheta)const ;
    double reachableQualityEnergy() const;

  private:
    flann::Index<flann::L2<double> > *poseFlannIndex ;
    Eigen::VectorXd isReachableFlagMatrix;
    std::vector<double> stepSize;


};


#endif
