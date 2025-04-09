#include "QuadricDecimationMesh.h"

const QuadricDecimationMesh::VisualizationMode QuadricDecimationMesh::QuadricIsoSurfaces =
    NewVisualizationMode("Quadric Iso Surfaces");

void QuadricDecimationMesh::Initialize() {
    // Allocate memory for the quadric array
    size_t numVerts = mVerts.size();
    mQuadrics.reserve(numVerts);
    std::streamsize width = std::cerr.precision();  // store stream precision
    for (size_t i = 0; i < numVerts; i++) {

        // Algrotihm steps

        // 1. Compute the Q matrices for all the initial vertices

        // Compute quadric for vertex i here
        mQuadrics.push_back(createQuadricForVert(i));

        // Calculate initial error, should be numerically close to 0

        glm::vec3 v0 = mVerts[i].pos;
        glm::vec4 v(v0[0], v0[1], v0[2], 1);
        glm::mat4 m = mQuadrics.back();

        // TODO CHECK
        float error = glm::dot(v, (m * v));
        std::cerr << std::scientific << std::setprecision(2) << error << " ";
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

    // 2. Select the valid pairs
    HalfEdge e1 = e(collapse->halfEdge);
    size_t v1 = e1.vert;                // v1 ------ e1 ------> v2
    size_t v2 = e(e1.pair).vert;    // v1 <-- e(e1.pair) -- v2

    // 3. Compute the optimal contraction target v_bar for each valid pair (v1, v2). The error
    //     v_bar_t * (Q1 + Q2) * v_bar of the target becomes the *cost* of contracting that pair
    glm::mat4 Q = mQuadrics[v1] + mQuadrics[v2]; // Q1 + Q2

    glm::mat4 Q_bar = Q;

    // Q_bar, found in section 4 of Surface Simplification Using Quadric Error Metrics
    Q_bar[0][3] = 0.0f;
    Q_bar[1][3] = 0.0f;
    Q_bar[2][3] = 0.0f;
    Q_bar[3][3] = 1.0f;


    glm::vec4 v_bar;

    // Needs to check if Q_bar is invertible with det(Q_bar) != 0
    if (abs(glm::determinant(Q_bar)) > 0.0000000001f) {

        v_bar = glm::inverse(Q_bar) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f); // Q_bar is invertible

    } else{ // Q_bar is not invertible
        // Extract both vertex positions, and a midpoint between them
        glm::vec4 pos_v1 = glm::vec4(v(v1).pos, 1.0f);
        glm::vec4 pos_v2 = glm::vec4(v(v2).pos, 1.0f);
        glm::vec4 pos_mid = ((pos_v1 + pos_v2) / 2.0f);

        // Construct a list of canditades for the smallest errors, will be looped through with range based loop
        std::vector<glm::vec4> candidates{pos_v1, pos_v2, pos_mid};

        // Calculate smallest error
        float smallest = std::numeric_limits<float>::infinity(); // Positive infinity, so the first value will be the smallest

        for (const auto& candidate : candidates) {
            float error = glm::dot(candidate, Q * candidate); // Calculate error 
            if (error < smallest) {
                smallest = error;
                v_bar = candidate;
            }
        }
    }
    // 4. Place all the pairs in a heap keyed on cost with the minimum cost pair at the top
    collapse->position = glm::vec3(v_bar.x, v_bar.y, v_bar.z);  // Set the target position for this collapse
    collapse->cost = glm::dot(v_bar, Q * v_bar);

    //std::cerr << "computeCollapse in QuadricDecimationMesh not implemented.\n";
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
    glm::mat4 Q({0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f},
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

    glm::vec3 v1 = v(indx).pos; // Vertex v1, extracted from edge e1
    glm::vec3 normal1 = f(indx).normal; // Extract the normal from the calling face

    float d = -glm::dot(normal1, v1);

    glm::vec4 plane(normal1, d);  // Extend d into a plane
    glm::mat4 quadratic = glm::outerProduct(plane, plane); // Compute quadratic

    return quadratic;
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
