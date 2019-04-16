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

        Vector3D normal(0,0,0);         // initialize a vector to store your normal sum
        HalfedgeCIter h = halfedge();   // Since we're in a Vertex, this returns a halfedge
        HalfedgeCIter hConst = h;
        double sum = 0.;

        do
        {
            //要取得是平面的normal，而不是点的normal
            Vector3D nighborNormal(h->face()->normal());
            //下面是两个向量，然后做叉乘
            Vector3D v1(h->vertex()->position - (h->twin()->vertex())->position);
            Vector3D v2(h->vertex()->position - (h->twin()->next()->twin()->vertex()->position));
            double tmpWeight = cross(v1, v2).norm();
            sum += tmpWeight;
            //三角形面积的两倍（平行四边形的面积）乘上normal累加上去
            normal += nighborNormal * tmpWeight;
            //std::cout<< "暂时输出normal为" << normal<<std::endl;
            h = h->twin()->next();
        }while(h != hConst);

        //最后归一化
        normal /= sum;
        return normal;
    }

    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0)
    {
        // TODO Part 4.
        // TODO This method should flip the given edge and return an iterator to the flipped edge.
        // 首先获取所有的可能改变的元素

        FaceIter f0 = e0->halfedge()->face();
        FaceIter f1 = e0->halfedge()->twin()->face();

        if(f0->isBoundary() || f1->isBoundary())
        {
            return e0;
        }

        HalfedgeIter h0 = e0->halfedge();
        HalfedgeIter h1 = h0->next();
        HalfedgeIter h2 = h1->next();
        HalfedgeIter h3 = h0->twin();
        HalfedgeIter h4 = h3->next();
        HalfedgeIter h5 = h4->next();
        HalfedgeIter h6 = h1->twin();
        HalfedgeIter h7 = h2->twin();
        HalfedgeIter h8 = h4->twin();
        HalfedgeIter h9 = h5->twin();

        VertexIter v0 = h0->vertex();
        VertexIter v1 = h3->vertex();
        VertexIter v2 = h6->vertex();
        VertexIter v3 = h8->vertex();

        EdgeIter e1 = h1->edge();
        EdgeIter e2 = h2->edge();
        EdgeIter e3 = h4->edge();
        EdgeIter e4 = h5->edge();

        //接着重新分配
        //想了一下，分配顺序应该没有关系
        h0->next() = h1;
        h0->twin() = h3;
        h0->face() = f0;
        h0->vertex() = v3;
        h0->edge() = e0;

        h1->next() = h2;
        h1->twin() = h7;
        h1->face() = f0;
        h1->vertex() = v2;
        h1->edge() = e2;

        h2->next() = h0;
        h2->twin() = h8;
        h2->face() = f0;
        h2->vertex() = v0;
        h2->edge() = e3;

        h3->next() = h4;
        h3->twin() = h0;
        h3->face() = f1;
        h3->vertex() = v2;
        h3->edge() = e0;

        h4->next() = h5;
        h4->twin() = h9;
        h4->face() = f1;
        h4->vertex() = v3;
        h4->edge() = e4;

        h5->next() = h3;
        h5->twin() = h6;
        h5->face() = f1;
        h5->vertex() = v1;
        h5->edge() = e1;

        h6->next() = h6->next();
        h6->twin() = h5;
        h6->face() = h6->face();
        h6->vertex() = v2;
        h6->edge() = e1;

        h7->next() = h7->next();
        h7->twin() = h1;
        h7->face() = h7->face();
        h7->vertex() = v0;
        h7->edge() = e2;

        h8->next() = h8->next();
        h8->twin() = h2;
        h8->face() = h8->face();
        h8->vertex() = v3;
        h8->edge() = e3;

        h9->next() = h9->next();
        h9->twin() = h4;
        h9->face() = h9->face();
        h9->vertex() = v1;
        h9->edge() = e4;

        v0->halfedge() = h2;
        v1->halfedge() = h5;
        v2->halfedge() = h3;
        v3->halfedge() = h0;

        e0->halfedge() = h0;
        e1->halfedge() = h5;
        e2->halfedge() = h1;
        e3->halfedge() = h2;
        e4->halfedge() = h4;

        f0->halfedge() = h0;
        f1->halfedge() = h3;

        return e0;
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0)
    {
        // TODO Part 5.
        // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
        // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.

        FaceIter f0 = e0->halfedge()->face();
        FaceIter f1 = e0->halfedge()->twin()->face();

        if(f0->isBoundary() || f1->isBoundary())
        {
            return newVertex();
        }

        HalfedgeIter h0 = e0->halfedge();
        HalfedgeIter h1 = h0->next();
        HalfedgeIter h2 = h1->next();
        HalfedgeIter h3 = h0->twin();
        HalfedgeIter h4 = h3->next();
        HalfedgeIter h5 = h4->next();
        HalfedgeIter h6 = h1->twin();
        HalfedgeIter h7 = h2->twin();
        HalfedgeIter h8 = h4->twin();
        HalfedgeIter h9 = h5->twin();

        VertexIter v0 = h0->vertex();
        VertexIter v1 = h3->vertex();
        VertexIter v2 = h6->vertex();
        VertexIter v3 = h8->vertex();

        EdgeIter e1 = h1->edge();
        EdgeIter e2 = h2->edge();
        EdgeIter e3 = h4->edge();
        EdgeIter e4 = h5->edge();

        //创建新的halfElement

        //三条新的edge
        EdgeIter e5 = newEdge();
        EdgeIter e6 = newEdge();
        EdgeIter e7 = newEdge();

        //六个新的halfedge
        HalfedgeIter h10 = newHalfedge();
        HalfedgeIter h11 = newHalfedge();
        HalfedgeIter h12 = newHalfedge();
        HalfedgeIter h13 = newHalfedge();
        HalfedgeIter h14 = newHalfedge();
        HalfedgeIter h15 = newHalfedge();

        //两个新的face
        FaceIter f2 = newFace();
        FaceIter f3 = newFace();

        //一个新的vertex
        VertexIter v4 = newVertex();
        v4->position = (v1->position + v0->position) / 2;

        //重新分配
        h0->setNeighbors(h1, h3, v4, e0, f0);

        h1->setNeighbors(h10, h6, v1, e1, f0);

        h2->setNeighbors(h14, h7, v2, e2, f3);

        h3->setNeighbors(h11, h0, v1, e0, f1);

        h4->setNeighbors(h12, h8, v0, e3, f2);

        h5->setNeighbors(h3, h9, v3, e4, f1);

        h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());

        h7->setNeighbors(h7->next(), h2, v0, e2, h7->face());

        h8->setNeighbors(h8->next(), h4, v3, e3, h8->face());

        h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());

        h10->setNeighbors(h0, h15, v2, e5, f0);

        h11->setNeighbors(h5, h12, v4, e7, f1);

        h12->setNeighbors(h13, h11, v3, e7, f2);

        h13->setNeighbors(h4, h14, v4, e6, f2);

        h14->setNeighbors(h15, h13, v0, e6, f3);

        h15->setNeighbors(h2, h10, v4, e5, f3);

        e0->halfedge() = h0;
        e1->halfedge() = h1;
        e2->halfedge() = h2;
        e3->halfedge() = h4;
        e4->halfedge() = h5;
        e5->halfedge() = h10;
        e6->halfedge() = h14;
        e7->halfedge() = h11;

        f0->halfedge() = h0;
        f1->halfedge() = h3;
        f2->halfedge() = h4;
        f3->halfedge() = h2;

        v0->halfedge() = h4;
        v1->halfedge() = h3;
        v2->halfedge() = h2;
        v3->halfedge() = h5;
        v4->halfedge() = h0;

        return v4;
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
