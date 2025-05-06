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

        float dx = mLS->GetDx();
        glm::vec3 v = glm::abs(mVectorField->GetMaxValue());
        float vMax = glm::max(glm::max(v.x, v.y), v.z);
        float timeStep = dx / vMax;
        return timeStep * 0.9;
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

        glm::vec3 vField = mVectorField->GetValue(x, y, z);

        glm::vec3 grad;

        if (vField.x < 0.0f) {
            grad.x = mLS->DiffXp(i, j, k);
        } 
        else {
            grad.x = mLS->DiffXm(i, j, k);
        }

        if (vField.y < 0.0f) {
            grad.y = mLS->DiffYp(i, j, k);
        } else {
            grad.y = mLS->DiffYm(i, j, k);
        }

        if (vField.z < 0.0f) {
            grad.z = mLS->DiffZp(i, j, k);
        } else {
            grad.z = mLS->DiffZm(i, j, k);
        }

        float changeRate = -1.0 * glm::dot(vField, grad);
        return changeRate;
    }
};