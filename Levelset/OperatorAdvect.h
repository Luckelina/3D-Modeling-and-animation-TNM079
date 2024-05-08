#pragma once

#include "Levelset/LevelSetOperator.h"
#include "Math/Function3D.h"

/*! \brief A level set operator that does external advection
 *
 * This class implements level set advectionr in an external vector field by the
 * PDE
 *
 *  \f$
 *  \dfrac{\partial \phi}{\partial t} + \mathbf{V}(\mathbf{x})\cdot \nabla \phi
 * = 0 \f$
 */
//! \lab4 Implement advection in external vector field
class OperatorAdvect : public LevelSetOperator {
protected:
    Function3D<glm::vec3>* mVectorField;

public:
    OperatorAdvect(LevelSet* LS, Function3D<glm::vec3>* vf)
        : LevelSetOperator(LS), mVectorField(vf) {}

    virtual float ComputeTimestep() {
        // Compute and return a stable timestep
        // (Hint: Function3D::GetMaxValue())
        glm::vec3 maxVelocity = abs(mVectorField->GetMaxValue()); //Vx or Vy or Vz
        float dx = mLS->GetDx();  // Dx=Dy=Dz
        float reduce = 0.9f;
                                               
        return (reduce*dx)/(std::max({maxVelocity.x, maxVelocity.y, maxVelocity.z}));
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

            IntegrateEuler(dt);
            // IntegrateRungeKutta(dt);
        }
    }

    virtual float Evaluate(size_t i, size_t j, size_t k) {
        // Compute the rate of change (dphi/dt)

        // Remember that the point (i,j,k) is given in grid coordinates, while
        // the velocity field used for advection needs to be sampled in
        // world coordinates (x,y,z). You can use LevelSet::TransformGridToWorld()
        // for this task.
        float x = i;
        float y = j;
        float z = k;
        mLS->TransformGridToWorld(x, y, z);

        glm::vec3 velocity = mVectorField->GetValue(x,y,z);
        glm::vec3 gradient{0.0f,0.0f,0.0f};
        // Upwind scheme: approximate the derivative using values that are "upwind" of the current point. 
        // For a positive flow direction, you look at the value of the function at points where the flow is coming from
        // (i.e., points with a lower x-coordinate). This ensures that you're using information from where the flow is coming,
        // which is more relevant for predicting how the function changes in the direction of the flow.

        size_t a = i;
        size_t b = j;
        size_t c = k;



        if(velocity[0] > 0.0f){ // x 
            gradient[0] = mLS->DiffXm(a,b,c);
        }
        else { gradient[0] = mLS->DiffXp(a,b,c); }

        if(velocity[1] > 0.0f){ // y
            gradient[1] = mLS->DiffYm(a,b,c);
        }
        else { gradient[1] = mLS->DiffYp(a,b,c); }

        if(velocity[2] > 0.0f){ // z
            gradient[2] = mLS->DiffZm(a,b,c);
        }
        else { gradient[2] = mLS->DiffZp(a,b,c); }

        
        return -1.0f * glm::dot(velocity, gradient);
    }
};