/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 *   Gunnar Johansson (gunnar.johansson@itn.liu.se)
 *   Ken Museth (ken.museth@itn.liu.se)
 *   Michael Bang Nielsen (bang@daimi.au.dk)
 *   Ola Nilsson (ola.nilsson@itn.liu.se)
 *   Andreas Sderstrm (andreas.soderstrom@itn.liu.se)
 *
 *************************************************************************************************/
#pragma once

#include "Levelset/LevelSetOperator.h"

/*! \brief A level set operator that does erosion or dilation.
 *
 * This class implements level set propagation in the normal direction
 * as defined by
 *  \f$
 *  \dfrac{\partial \phi}{\partial t}+ F(\mathbf{x})|\nabla \phi| = 0
 *  \f$
 * where the sign of F dictates erosion (c<0), or dilation (c>0).
 */
//! \lab4 Implement erosion and dilation
class OperatorDilateErode : public LevelSetOperator {
protected:
    //! The constant speed function
    float mF;

public:
    OperatorDilateErode(LevelSet* LS, float f) : LevelSetOperator(LS), mF(f) {}

    virtual float ComputeTimestep() {
        // Compute and return a stable timestep
        float gridSpacing = mLS->GetDx(); //Uniform grid spacing! Dx=Dy=Dz
        return std::min(gridSpacing / std::abs(mF), 1.0f); //1.0f limits timestep to 1.
    }

    virtual void Propagate(float time) {
        // Determine timestep for stability
        float dt = ComputeTimestep();

        // Propagate level set with stable timestep dt
        // until requested time is reached
        for (float elapsed = 0.f; elapsed < time;) {
            if (dt > time - elapsed) {
                dt = time - elapsed;
            }
            elapsed += dt;

            // Integrate level set function in time using Euler integration
            IntegrateEuler(dt);
            // IntegrateRungeKutta(dt);
        }
    }

    virtual float Evaluate(size_t i, size_t j, size_t k) {
        // Compute the rate of change (dphi/dt)
        float ddx2, ddy2, ddz2;  // Squares of the partial derivatives in each direction
        float signF;
        //if (mF > 0) {  // Determine the sign of the speed function
        //    signF = 1.0f;
        //}else {
        //    signF = -1.0f;  
        //}

        // Call Godunov method to calculate the correct gradient squares
        Godunov(i, j, k, mF, ddx2, ddy2, ddz2);

        // Calculate the norm of the gradient
        float gradNorm = -1*mF*std::sqrt(ddx2 + ddy2 + ddz2);
        return gradNorm;
    }
};
