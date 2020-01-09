/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef POSEMATCH_H_
#define POSEMATCH_H_

#include <Eigen/Core>
#include "Ferns.h"

/** @brief // ? 由于 Local Loop 和 Graph Loop 带来的配准对齐的约束? */
class PoseMatch
{
    public:
        PoseMatch(int firstId,
                  int secondId,
                  const Eigen::Matrix4f & first,
                  const Eigen::Matrix4f & second,
                  const std::vector<Ferns::SurfaceConstraint> & constraints,
                  const bool & fern)
     : firstId(firstId),
       secondId(secondId),
       first(first),
       second(second),
       constraints(constraints),
       fern(fern)
    {}

    int firstId;                                        ///? 什么的id?
    int secondId;                                       ///? 同上?
    Eigen::Matrix4f first;
    Eigen::Matrix4f second;
    std::vector<Ferns::SurfaceConstraint> constraints;  ///? 约束, 一对点一对点
    bool fern;                                          ///< 是否是由 Global Loop 导致的
};

#endif /* POSEMATCH_H_ */
