#include <Subdivision/UniformCubicSplineSubdivisionCurve.h>
#include <glm.hpp>
#include <gtc/type_ptr.hpp>

UniformCubicSplineSubdivisionCurve::UniformCubicSplineSubdivisionCurve(
    const std::vector<glm::vec3>& joints, glm::vec3 lineColor, float lineWidth)
    : mCoefficients(joints), mControlPolygon(joints) {
    this->mLineColor = lineColor;
    this->mLineWidth = lineWidth;
}

void UniformCubicSplineSubdivisionCurve::Subdivide() {
    // Allocate space for new coefficients
    std::vector<glm::vec3> newc;
    assert(mCoefficients.size() > 4 && "Need at least 5 points to subdivide");

    // Implement the subdivision scheme for a natural cubic spline here
    newc.reserve(mCoefficients.size() * 2 - 1);  // Prepare space for new coefficients




    float coeff = (1.0f / 8.0f);
    
    glm::vec3 ci = mCoefficients[0]; 
    glm::vec3 c_next = mCoefficients[1];
    newc.push_back(ci); //Add first boundary
    newc.push_back(coeff * (4.0f * ci + 4.0f * c_next)); //Add first subdivision

    
    for (int i = 1; i < mCoefficients.size()-1; i++) { //Add everything inbetween

        glm::vec3 c_prev = mCoefficients[i - 1];
        glm::vec3 ci = mCoefficients[i];
        glm::vec3 c_next = mCoefficients[i + 1];

        glm::vec3 ci_new = coeff * (c_prev + 6.0f * ci + c_next);
        glm::vec3 ci_next_new = coeff * (4.0f * ci + 4.0f * c_next);

        newc.push_back(ci_new);
        newc.push_back(ci_next_new);
        
    }
    newc.push_back(mCoefficients[mCoefficients.size()-1]); //Add last boundary


    // If 'mCoefficients' had size N, how large should 'newc' be? Perform a check
    // here!
    bool largeCheck = (newc.size() == 2 * mCoefficients.size() - 1);
    assert(largeCheck && "Incorrect number of new coefficients!");

    mCoefficients = newc;
}

void UniformCubicSplineSubdivisionCurve::Render() {
    // Apply transform
    glPushMatrix();  // Push modelview matrix onto stack

    // Convert transform-matrix to format matching GL matrix format
    // Load transform into modelview matrix
    glMultMatrixf(glm::value_ptr(mTransform));

    mControlPolygon.Render();

    // save line point and color states
    glPushAttrib(GL_POINT_BIT | GL_LINE_BIT | GL_CURRENT_BIT);

    // draw segments
    glLineWidth(mLineWidth);
    glColor3fv(glm::value_ptr(mLineColor));
    glBegin(GL_LINE_STRIP);
    // just draw the spline as a series of connected linear segments
    for (size_t i = 0; i < mCoefficients.size(); i++) {
        glVertex3fv(glm::value_ptr(mCoefficients.at(i)));
    }
    glEnd();

    // restore attribs
    glPopAttrib();

    glPopMatrix();

    GLObject::Render();
}
