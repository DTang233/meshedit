#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */


  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.

      vector<Vector2D> res;
      for(int i = 0; i < points.size()-1;i++){
          Vector2D pi = (1-t)*points[i] + t*points[1+i];
          res.push_back(pi);
      }
      
    return res;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
        vector<Vector3D> res;
        for(int i = 0; i < points.size()-1;i++){
            Vector3D pi = (1-t)*points[i] + t*points[1+i];
            res.push_back(pi);
        }

      return res;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      vector<Vector3D> rem = evaluateStep(points, t);
      
      if (rem.size() == 1){
          return rem[0];
      }
      else{
          return evaluate1D(rem, t);
      }
    
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.

      // iterate through row
      vector<Vector3D> ns;
      for(int i = 0; i < controlPoints.size();i++){
          Vector3D n = evaluate1D(controlPoints[i],u);
          ns.push_back(n);
      }
      Vector3D res = evaluate1D(ns,v);
      
    return res;
  }


  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
      
      //h->twin()->next() to get the edge
      HalfedgeCIter h = halfedge();
      Vector3D res = Vector3D(0, 0, 0);
      do{
          Vector3D v0 = h->vertex()->position;
          HalfedgeCIter ht = h->twin();
          Vector3D v1 = ht->vertex()->position;
          HalfedgeCIter hn = ht->next();
          Vector3D v2 = hn->twin()->vertex()->position;
          Vector3D vab = v1 - v0;
          Vector3D vac = v2 - v0;
          
          float area = (sqrt(pow((vab.y*vac.z-vab.z*vac.y),2)+pow((vab.z*vac.x-vab.x*vac.z),2)+pow((vab.x*vac.y-vab.y*vac.x),2)))/2;
          
          Vector3D norm = cross(vac, vab);
          
          res += norm * area;
          h = hn;
//          cout << res;
          
      }while(h != halfedge());
      
      res.normalize();
    return res;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    // Never flip boundary edge.
      
    //original edges
      HalfedgeIter bc = e0->halfedge();
      HalfedgeIter cb = bc->twin();
       //mwybe also check twin
      if (bc->isBoundary() || cb->isBoundary()){
          return EdgeIter();
      }
      FaceIter f0 = bc->face();
      
      FaceIter f1 = cb->face();
      
      HalfedgeIter ca = bc->next();
      HalfedgeIter ac = ca->twin();
      
      HalfedgeIter ab = ca->next();
      HalfedgeIter ba = ab->twin();
      
      HalfedgeIter bd = cb->next();
      HalfedgeIter db = bd->twin();
      
      HalfedgeIter dc = bd->next();
      HalfedgeIter cd = dc->twin();
      
      
      //original vertices
      VertexIter a = ab->vertex();
      VertexIter b = bd->vertex();
      VertexIter c = cb->vertex();
      VertexIter d = dc -> vertex();
     
      //set vertices
      a->halfedge() = bc;
      d->halfedge() = cb;
    
      c->halfedge() = ca;
      b->halfedge() = bd;
     
      //ad.setNeighbors(<#HalfedgeIter next#>, <#HalfedgeIter twin#>, <#VertexIter vertex#>,
      
      f0->getFace()->halfedge() = dc;
      f1->getFace()->halfedge() = ab;
    //For each half-edge, set its next, twin, vertex, edge, and face pointer to the correct
//
      ca->setNeighbors(bc, ac, c, ca->edge(), f0);
      bc->setNeighbors(dc, cb, a, bc->edge(), f0);
      dc->setNeighbors(ca, cd, d, dc->edge(), f0);
      
      cb->setNeighbors(ab, bc, d, cb->edge(), f1);
      ab->setNeighbors(bd, ba, a, ab->edge(), f1);
      bd->setNeighbors(cb, db, b, bd->edge(), f1);
      //set faces' new half edge

//
    return bc->edge();
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      HalfedgeIter bc = e0->halfedge();
      HalfedgeIter cb = bc->twin();
       //mwybe also check twin
      if (bc->isBoundary() || cb->isBoundary()){
          return VertexIter();
      }
      FaceIter f0 = bc->face();
      FaceIter f1 = cb->face();
      
      HalfedgeIter ca = bc->next();
      HalfedgeIter ac = ca->twin();
      
      HalfedgeIter ab = ca->next();
      HalfedgeIter ba = ab->twin();
      
      HalfedgeIter bd = cb->next();
      HalfedgeIter db = bd->twin();
      
      HalfedgeIter dc = bd->next();
      HalfedgeIter cd = dc->twin();
    
      //original vertices
      VertexIter a = ab->vertex();
      VertexIter b = bd->vertex();
      VertexIter c = cb->vertex();
      VertexIter d = dc -> vertex();
      
      //new elements
      //new point
      
      VertexIter m = newVertex();
      m->position = ((c->position)+(b->position))/2;
     
      FaceIter f2 = newFace();
      FaceIter f3 = newFace();

      EdgeIter e1 = newEdge();
      EdgeIter e2 = newEdge();
      EdgeIter e3 = newEdge();
      
      HalfedgeIter am = newHalfedge();
      HalfedgeIter ma = newHalfedge();
      HalfedgeIter bm = newHalfedge();
      HalfedgeIter mb = newHalfedge();
      HalfedgeIter md = newHalfedge();
      HalfedgeIter dm = newHalfedge();
      
      //set halfedge for points
      a->halfedge() = ab;
      b->halfedge() = bd;
      d -> halfedge() = dc;
      c ->halfedge() = ca;
      m->halfedge() = bc;
      
      //set edge
      e1->halfedge() = am;
      e2->halfedge() = md;
      e3->halfedge() = mb;

      
      //set faces' halfedge
      f0->halfedge() = ca;
      f1 -> halfedge() = dc;
      f2 -> halfedge() = ab;
      f3 ->halfedge() = bd;
      
      //set halfeage
      bc->setNeighbors(ca, cb, m, bc->edge(), f0);
      ca->setNeighbors(am, ac, c, ca->edge(), f0);
      am->setNeighbors(bc, ma, a, e1, f0);
      
      cb->setNeighbors(md, bc, c, cb->edge(), f1);
      md->setNeighbors(dc, dm, m, e2, f1);
      dc->setNeighbors(cb, cd, d, dc->edge(), f1);
      
      bm->setNeighbors(ma, mb, b, e3, f2);
      ma->setNeighbors(ab, am, m, e1, f2);
      ab->setNeighbors(bm, ba, a, ab->edge(), f2);
      
      mb->setNeighbors(bd, bm, m, e3, f3);
      bd->setNeighbors(dm, db, b, bd->edge(), f3);
      dm->setNeighbors(mb, md, d, e2, f3);
      

//      cout << m->position;

      
    return m;
  }



bool ifFlip(HalfedgeIter e){
    if(!(e->vertex()->isNew && e->twin()->vertex()->isNew)){
        return true;
    }
    return false;
}


  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
      //difference betwen citer and iter??

//      EdgeIter e0 = mesh.edgesBegin();
//      if(e0->isBoundary()){
//          return;
//      }
      
      //Calc the pos for new v
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
          HalfedgeIter h = e->halfedge();
          Vector3D a_pos = h->vertex()->position;
          Vector3D b_pos = h->twin()->vertex()->position;
          Vector3D c_pos = h->twin()->next()->twin()->vertex()->position;
          Vector3D d_pos = h->next()->twin()->vertex()->position;
         
          
          //  3/8 * (A + B) + 1/8 * (C + D)
          e->newPosition = ((float)3/(float)8)*(a_pos+b_pos) + ((float)1/(float)8)*(c_pos + d_pos);
          e->isNew = false;
      }

      //Calc the pos for old v
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          
          float u;
          int n = v->degree();
          if (n==3){
              u = (float)3/(float)16;
          }
          else{
              u = (float)3/((float)8*(float)n);
          }
          //ierate thourgh neighbors
          Vector3D nsum = Vector3D(0,0,0);
          HalfedgeIter h = v->halfedge();
          do{
              VertexIter vn = h->twin()->vertex();
              nsum += vn->position;
              h = h->next()->next()->twin();
              
          }while(h != v->halfedge());
          //(1 - n * u) * original_position + u * original_neighbor_position_sum
          v->newPosition = (1-n*u) * (v->position) + u * nsum;
          v->isNew = false;
      }
      
      //split the edge
      std::vector<EdgeIter> new_es;
//      std::vector<VertexIter> new_vs;
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
          if(!e->isNew){
              VertexIter new_v = mesh.splitEdge(e);
              new_v->isNew = true;
              new_v->position = e->newPosition;
              HalfedgeIter h = new_v -> halfedge();
              EdgeIter new_e0 = h->edge();
              EdgeIter new_e1 = h->next()->next()->edge();
              EdgeIter new_e2 = h->twin()->next()->edge();
              EdgeIter new_e3 = h->twin()->next()->twin()->next()->edge();
              new_e0->isNew = true;
              new_e1->isNew = true;
              new_e2->isNew = true;
              new_e3->isNew = true;

              new_es.push_back(new_e1);
              new_es.push_back(new_e2);

          }
      }
      
      for (int i = 0; i < new_es.size(); i++){
          HalfedgeIter h = new_es[i]->halfedge();
          if(h->vertex()->isNew != h->twin()->vertex()->isNew){
               mesh.flipEdge(new_es[i]);
          }

      }
//
      for(VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++){
          if(!v->isNew){
              v->position = v->newPosition;
          }
      }
      
    
  }
}
