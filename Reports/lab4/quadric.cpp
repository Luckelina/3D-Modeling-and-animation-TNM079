#include <Geometry/Quadric.h>

Quadric::Quadric(const glm::mat4& q) : mQuadric(q) {}

Quadric::~Quadric() {}

/*!
 * Evaluation of world coordinates are done through either transformation
 * of the world-coordinates by mWorld2Obj, or transformation of the quadric
 * coefficient matrix by GetTransform() ONCE (see Section 2.2 in lab text).
 */
float Quadric::GetValue(float x, float y, float z) const {
    TransformW2O(x, y, z);
    glm::vec4 p(x, y, z, 1.0f);
    return glm::dot(p, mQuadric*p);
}

/*!
 * Use the quadric matrix to evaluate the gradient.
 */
glm::vec3 Quadric::GetGradient(float x, float y, float z) const {
    TransformW2O(x, y, z);
    glm::vec4 p(x, y, z, 1.0f);
    glm::vec4 grad = 2.0f * mQuadric * p;
    return glm::vec3(grad.x, grad.y, grad.z);
}
