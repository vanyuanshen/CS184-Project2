/**
 * 这个文件把我们要实现的各种类的函数都放在这里了
 */
#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

    void BezierCurve::evaluateStep()
    {
        // TODO Part 1.
        // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
        // Store all of the intermediate control points into the 2D vector evaluatedLevels.
        int maxLevel = numControlPoints;
        int currentLevel  = evaluatedLevels.size();
        if(currentLevel == maxLevel)
            return;
        else
        {
            std::vector<Vector2D> currentLevelPoints = evaluatedLevels[evaluatedLevels.size()-1];
            int nextLevelSize = currentLevelPoints.size() - 1;
            std::vector<Vector2D> newLevelPoints(nextLevelSize);
            for(int i = 0; i < nextLevelSize; i++)
            {
                newLevelPoints[i] = currentLevelPoints[i] * (1 - t) + currentLevelPoints[i + 1] * t;
            }
            evaluatedLevels.push_back(newLevelPoints);
        }
    }


    Vector3D BezierPatch::evaluate(double u, double v) const
    {
        // TODO Part 2.
        // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
        // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
        // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
        // 现在第一次形成四条曲线用的参数u，第二次形成曲面用的参数v
//        std::vector<Vector3D> tmpControlPoints;
//        for(int numCurves = 0; numCurves < controlPoints.size(); numCurves++)
//        {
//            tmpControlPoints.push_back(evaluate1D(controlPoints[numCurves], u));
//        }
//        Vector3D finalPoint(evaluate1D(tmpControlPoints, v));
//        return finalPoint;
        std::vector<Vector3D> p;
        for(int i = 0; i < controlPoints.size(); ++i){
            p.push_back(evaluate1D(controlPoints[i], u));
        }
        return evaluate1D(p,v);
    }

    //返回1D贝塞尔曲线上的一个点
    Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
    {
        // TODO Part 2.
        // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
        // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.

        //我的解决方案
        std::vector<Vector3D> tmpControlPoints(points);
        std::vector<Vector3D> finalControlPoints;

        while (tmpControlPoints.size() >= 2)
        {
            finalControlPoints.clear();
            for (int j = 1; j < tmpControlPoints.size(); j++) {
                Vector3D tmp(tmpControlPoints[j - 1] * (1 - t) + tmpControlPoints[j] * t);
                finalControlPoints.push_back(tmp);
            }
            tmpControlPoints.clear();
            tmpControlPoints.assign(finalControlPoints.begin(), finalControlPoints.end());
        }
        Vector3D result;
        if (!finalControlPoints.empty())
        {
            result = finalControlPoints[0];
        }
        return result;

        //别人的解决方案
//        std::vector<std::vector<Vector3D> > evaluatedLevels;
//        evaluatedLevels.push_back(points);
//        while(evaluatedLevels.size() != points.size()){
//            std::vector<Vector3D> newLevel;
//            for(int i = 0; i < evaluatedLevels.back().size(); ++i){
//                newLevel.push_back(Vector3D(evaluatedLevels.back()[i] * (1-t) + evaluatedLevels.back()[i+1] * t));
//            }
//            evaluatedLevels.push_back(newLevel);
//        }
//
//        return evaluatedLevels.back()[0];
    }


    Vector3D Vertex::normal(void) const
    {
        // TODO Part 3.
        // TODO Returns an approximate unit normal at this vertex, computed by
        // TODO taking the area-weighted average of the normals of neighboring
        // TODO triangles, then normalizing.
        return Vector3D();
    }

    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0)
    {
        // TODO Part 4.
        // TODO This method should flip the given edge and return an iterator to the flipped edge.
        return e0;
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0)
    {
        // TODO Part 5.
        // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
        // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
        return newVertex();
    }


    void MeshResampler::upsample(HalfedgeMesh &mesh)
    {
        // TODO Part 6.
        // This routine should increase the number of triangles in the mesh using Loop subdivision.
        // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
        // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
        // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
        // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
        // the new mesh based on the values we computed for the original mesh.


        // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
        // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
        // TODO a vertex of the original mesh.


        // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.


        // TODO Next, we're going to split every edge in the mesh, in any order.  For future
        // TODO reference, we're also going to store some information about which subdivided
        // TODO edges come from splitting an edge in the original mesh, and which edges are new,
        // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
        // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
        // TODO just split (and the loop will never end!)


        // TODO Now flip any new edge that connects an old and new vertex.


        // TODO Finally, copy the new vertex positions into final Vertex::position.

        return;
    }
}
