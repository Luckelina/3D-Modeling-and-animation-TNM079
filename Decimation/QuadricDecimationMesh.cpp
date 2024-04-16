#include "QuadricDecimationMesh.h"

const QuadricDecimationMesh::VisualizationMode QuadricDecimationMesh::QuadricIsoSurfaces =
    NewVisualizationMode("Quadric Iso Surfaces");

void QuadricDecimationMesh::Initialize() {
    // Allocate memory for the quadric array
    size_t numVerts = mVerts.size();
    mQuadrics.reserve(numVerts);
    std::streamsize width = std::cerr.precision();  // store stream precision
    for (size_t i = 0; i < numVerts; i++) {

        // Compute quadric for vertex i here
        mQuadrics.push_back(createQuadricForVert(i));

        // Calculate initial error, should be numerically close to 0

        glm::vec3 v0 = mVerts[i].pos;
        glm::vec4 v(v0[0], v0[1], v0[2], 1);
        glm::mat4 m = mQuadrics.back();

        // TODO CHECK
        float error = glm::dot(v, (m * v));
        // std::cerr << std::scientific << std::setprecision(2) << error << " ";
    }
    std::cerr << std::setprecision(width) << std::fixed;  // reset stream precision

    // Run the initialize for the parent class to initialize the edge collapses
    DecimationMesh::Initialize();
}

/*! \lab2 Implement the computeCollapse here */
/*!
 * \param[in,out] collapse The edge collapse object to (re-)compute,
 * DecimationMesh::EdgeCollapse
 */
void QuadricDecimationMesh::computeCollapse(EdgeCollapse* collapse) {
    // Compute collapse->position and collapse->cost here
    // based on the quadrics at the edge endpoints

    HalfEdge e1 = e(collapse->halfEdge);
    auto v1 = e1.vert;
    auto v2 = e(e1.next).vert;

    // Q is the sum of the quadrics at the edge endpoints
    glm::mat4 Q = mQuadrics[v1] + mQuadrics[v2];

    // Set the last row to (0, 0, 0, 1) for the minimization problem
    Q[0][3] = 0;
    Q[1][3] = 0;
    Q[2][3] = 0;
    Q[3][3] = 1;
    glm::vec4 v_bar;

   
    //Check if invertable det(Q) != 0
    if (abs(glm::determinant(Q)) > 0.0000000001f) {
    
    
        v_bar = glm::inverse(Q) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    
    
    
    } else {
    
        Q = mQuadrics[v1] + mQuadrics[v2];
        
        glm::vec3 v1Pos = v(v1).pos;
        glm::vec3 v2Pos = v(v2).pos;
        glm::vec3 midPos = ((v1Pos + v2Pos) * 0.5f);
       
        
        std::vector<glm::vec3> candidates{v1Pos, v2Pos, midPos};
       // glm::vec3 candidates[3] = {v1Pos, v2Pos, midPos};
        float minError = std::numeric_limits<float>::max();
        for (const auto& candidate : candidates) {
            glm::vec4 v_candidate = glm::vec4(candidate, 1.0f);
            float error = glm::dot(v_candidate, Q * v_candidate);
            if (error < minError) {
                minError = error;
                v_bar = v_candidate;
            }
        }
    }

    collapse->position = {v_bar.x,v_bar.y,v_bar.z};  // Set the target position for this collapse
    float c = glm::dot(v_bar, Q * v_bar);
    collapse->cost = c;

   // std::cerr << "computeCollapse in QuadricDecimationMesh not implemented.\n";
}

/*! After each edge collapse the vertex properties need to be updated */
void QuadricDecimationMesh::updateVertexProperties(size_t ind) {
    DecimationMesh::updateVertexProperties(ind);
    mQuadrics[ind] = createQuadricForVert(ind);
}

/*!
 * \param[in] indx vertex index, points into HalfEdgeMesh::mVerts
 */
glm::mat4 QuadricDecimationMesh::createQuadricForVert(size_t indx) const {
    glm::mat4 Q({0.0f, 0.0f, 0.0f, 0.0f}, 
                {0.0f, 0.0f, 0.0f, 0.0f}, 
                {0.0f, 0.0f, 0.0f, 0.0f},
                {0.0f, 0.0f, 0.0f, 0.0f});

    // The quadric for a vertex is the sum of all the quadrics for the adjacent
    // faces Tip: Matrix4x4 has an operator +=

    for (const auto& faceIndx : FindNeighborFaces(indx)) {
        Q += createQuadricForFace(faceIndx);
    }


    return Q;
}

/*!
 * \param[in] indx face index, points into HalfEdgeMesh::mFaces
 */
glm::mat4 QuadricDecimationMesh::createQuadricForFace(size_t indx) const {

    // Calculate the quadric (outer product of plane parameters) for a face
    // here using the formula from Garland and Heckbert

    HalfEdge e1 = e(f(indx).edge);
    // Vertex positions of the face
    glm::vec3 v1 = v(e1.vert).pos;
    glm::vec3 v2 = v(e(e1.next).vert).pos;
    glm::vec3 v3 = v(e(e1.prev).vert).pos;

    glm::vec3 normal2 = glm::normalize(glm::cross(v2 - v1, v3 - v1));

    glm::vec3 normal1 = f(indx).normal;
    float d = -glm::dot(normal1, v1); // ax + by + cz + d. This is the last component d.

    glm::vec4 plane(normal1, d);                           // Extended plane coefficients to 4D. 
    glm::mat4 quadric = glm::outerProduct(plane, plane);  // Compute quadric.
    //std::cout << "normal1:\n" << normal1.x << ", " << normal1.y << ", " << normal1.z << "\nnormal 2:\n" << normal2.x << ", " << normal2.y << ", " << normal2.z << std::endl;

/* glm::mat4 Q({normal.x*normal.x, normal.x * normal.y, normal.x * normal.z, normal.x * d}, 
            {normal.x*normal.y, normal.y * normal.y, normal.y * normal.z, normal.y * d}, 
            {normal.x*normal.z, normal.y * normal.z, normal.z * normal.z, normal.z * d},
            {normal.x * d, normal.y * d, normal.z * d, d * d}); */


    return quadric;
    //return glm::mat4();
}

void QuadricDecimationMesh::Render() {
    DecimationMesh::Render();

    glEnable(GL_LIGHTING);
    glMatrixMode(GL_MODELVIEW);

    if (mVisualizationMode == QuadricIsoSurfaces) {
        // Apply transform
        glPushMatrix();  // Push modelview matrix onto stack

        // Implement the quadric visualization here
        std::cout << "Quadric visualization not implemented" << std::endl;

        // Restore modelview matrix
        glPopMatrix();
    }
}
