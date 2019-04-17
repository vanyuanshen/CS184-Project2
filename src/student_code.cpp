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
            return e0->halfedge()->vertex();
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
        //这里创建的时候已经加入mesh的list里面去了，所以确实改变了mesh的list

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
        for(auto it = mesh.verticesBegin(); it!= mesh.verticesEnd(); it++)
        {
            //先计算新的位置
            it->isNew = false;
            Vector3D newPosition(0, 0, 0);
            float sum = 0;
            auto nit = it->halfedge()->twin();
            auto originNit = nit;
            auto n = it->degree();
            //注意！！！！浮点数，如果直接3/16是0
            float u = (n == 3) ? (3.0/16.0) : (3.0 / (8.0 * n));
            do{
                sum += u;
                newPosition = newPosition + u * (nit->vertex()->position);
                nit = nit->next()->twin();
            }while (nit != originNit);
            sum += 1 - n*u;
            newPosition = newPosition + (1.0 - n*u) * it->position;
            newPosition = newPosition / sum;
            it->newPosition = newPosition;
        }


        // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
        // 这一步应该是计算将要创建的新点的位置（差不多是中点，然后放在edge的newPosition里）
        // 对于边界情况，我觉得直接中点就完事了
        for(auto it = mesh.edgesBegin(); it!=mesh.edgesEnd(); it++)
        {
            //每个边上一个新的点
            Vector3D newPosition(0, 0, 0);
            if(it->halfedge()->face()->isBoundary() || it->halfedge()->twin()->face()->isBoundary())
            {
                //边界情况直接中点
                newPosition = it->halfedge()->vertex()->position / 2.0 + it->halfedge()->twin()->vertex()->position / 2.0;
            }
            else
            {
                Vector3D v0 = it->halfedge()->vertex()->position;
                Vector3D v1 = it->halfedge()->twin()->vertex()->position;
                Vector3D v2 = it->halfedge()->next()->twin()->vertex()->position;
                Vector3D v3 = it->halfedge()->twin()->next()->twin()->vertex()->position;
                //注意啦，这里也被round to zero了，记住浮点数
                newPosition = v0 * 3.0/8.0 + v1 * 3.0/8.0 + v2 * 1.0/8.0 + v3 * 1.0/8.0;
            }
            it->newPosition = newPosition;
        }


        // TODO Next, we're going to split every edge in the mesh, in any order.  For future
        // TODO reference, we're also going to store some information about which subdivided
        // TODO edges come from splitting an edge in the original mesh, and which edges are new,
        // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
        // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
        // TODO just split (and the loop will never end!)
        // 这里一开始我没想明白，还以为要重新实现不同的split，但实际上就是对于mesh的三个边使用上一个part中的split函数
        // 得到的就是ppt中的第一步结果，然后再flip就好啦
        // 问题：每次split和flip，mesh到底变不变？貌似是不变的
        // 解答：对，就是不变的，所以一个mesh一开始是可能就覆盖一个面（三角形）,后面可能就有多个面，边，点等等
        vector<EdgeIter> originEdgeList;
        auto copyIt = mesh.edgesBegin();
        do{
            auto newIt = copyIt;
            newIt->isNew = false;               //一开始遍历的时候每次都改成false
            originEdgeList.push_back(newIt);
            copyIt++;
        }while(copyIt != mesh.edgesEnd());

        //注意这里，it是EdgeIter的iterator，而*it才是EdgeIter类型
        int countIt = 0;
        for(auto it = originEdgeList.begin(); it != originEdgeList.end(); it++)
        {
            // 返回split之后的点
            countIt++;
            auto v = mesh.splitEdge(*it);
            // 注意isNew的修改,这里将除了最初的edge以外的新创建的edge全部改为true;
            v->isNew = true;
            // 同时这里需要把edge的newPosition赋值给新创建的点。（如果没有这一步，最后copy
            // newPosition 到 position的时候就为空了。
            v->newPosition = (*it)->newPosition;

            auto newHalfIt = v->halfedge();
//
//            auto originHalfIt = newHalfIt;
//            do {
//                newHalfIt->edge()->isNew = true;
//                newHalfIt = newHalfIt->twin()->next();
//            }while(newHalfIt != originHalfIt);

            newHalfIt->edge()->isNew = false;
            newHalfIt->twin()->next()->edge()->isNew = true;
            newHalfIt->twin()->next()->twin()->next()->edge()->isNew = false;
            newHalfIt->next()->next()->edge()->isNew = true;
        }


        // TODO Now flip any new edge that connects an old and new vertex.
        // 然后再flip
        for(auto it = mesh.edgesBegin(); it != mesh.edgesEnd(); it++)
        {
            // != 与异或的功能一致
            if(it->halfedge()->vertex()->isNew != it->halfedge()->twin()->vertex()->isNew )
            {
                if(it->isNew)
                    mesh.flipEdge(it);
            }
        }

        // TODO Finally, copy the new vertex positions into final Vertex::position.
        for(auto it = mesh.verticesBegin(); it != mesh.verticesEnd(); it++)
        {
            it->position = it->newPosition;
        }
    }
}
