#ifndef COLLISION_INTERVAL_H
#define COLLISION_INTERVAL_H
/* This file includes all the functions and structures for precise calculation of collision intervals.
 * All the code was taken from T.Walker implementation of HOG2: https://github.com/thaynewalker/hog2
*/
#include <math.h>
#include <vector>
#include <limits>
#include <iostream>
#include "const.h"
static const double TOLERANCE = 0.00000001;

inline bool fless(double a, double b) { return (a < b - TOLERANCE); }
inline bool fgreater(double a, double b) { return (a > b + TOLERANCE); }
inline bool fequal(double a, double b)
{ return (a >= b - TOLERANCE) && (a <= b+TOLERANCE); }
inline bool fleq(double a, double b) { return fless(a,b)||fequal(a,b); }
inline bool fgeq(double a, double b) { return fgreater(a,b)||fequal(a,b); }

inline double min(double a, double b) { return fless(a, b)?a:b; }
inline double max(double a, double b) { return fless(a, b)?b:a; }

class Vector2D {
  public:
    Vector2D(Vector2D const& v):Vector2D(v.x,v.y){}
    //Vector2D(Point_2 const& p):Vector2D(p[0],p[1]){};
    Vector2D(double _x,double _y):x(_x),y(_y) {/*Normalize();*/}
    Vector2D():x(0),y(0) {}
    void Set(double _x, double _y) { x=_x; y=_y; /*Normalize();*/}

    void SetUpdateTime(double t) {}
    double GetUpdateTime() {return 0.0;}

    void SetAccessTime(double t) {}
    double GetAccessTime() {return 0.0;}
    //operator Point_2()const{return Point_2(x,y);}

    bool operator==(const Vector2D &rhs)const{return (fequal(x,rhs.x)&&fequal(y,rhs.y));}
    bool operator<(const Vector2D &rhs)const{return fequal(x,rhs.x)?fless(y,rhs.y):fless(x,rhs.x);}

    // Dot product
    inline double operator *(Vector2D const& other)const{return x * other.x + y * other.y;}
    inline Vector2D operator -(Vector2D const& other)const{return Vector2D(x-other.x,y-other.y);}
    inline Vector2D operator +(double s)const{return Vector2D(x+s,y+s);}
    inline void operator -=(Vector2D const& other){x-=other.x;y-=other.y;}
    // Negation
    inline Vector2D operator -()const{return Vector2D(-x,-y);}
    // Slope angle of this vector
    inline double atan()const{ return atan2(y,x); }
    // Square
    inline double sq()const{ return (*this) * (*this); }
    inline double len()const{ return sqrt(sq()); }
    inline double cross(Vector2D const& b)const{return x*b.y-y*b.x;}

    // Vector that is perpendicular to this vector
    inline Vector2D perp()const{return Vector2D(y,-x);}

    inline double min()const{return x<y?x:y;}
    inline double max()const{return x>y?x:y;}
    // Project an (unclosed) polygon onto this line (a normalized axis)
    inline Vector2D projectPolyOntoSelf(std::vector<Vector2D> const& poly)const{
      double min(poly[0]*(*this));
      double max=min;
      for(unsigned i(1); i<poly.size(); ++i){
        double proj(poly[i]*(*this));
        if(fless(proj,min)) min = proj;
        if(fgreater(proj,max)) max = proj;
      }
      return Vector2D(min,max);
    }

    friend Vector2D operator /(const Vector2D& vec, const double num)
    {
      return Vector2D(vec.x / num, vec.y / num);
    }

    friend Vector2D operator *(const Vector2D& vec, const double num)
    {
      return Vector2D(vec.x * num, vec.y * num);
    }

    friend Vector2D operator *(const double num, const Vector2D& vec)
    {
      return Vector2D(vec.x * num, vec.y * num);
    }

    friend Vector2D operator +(const Vector2D& v1, const Vector2D& v2)
    {
      return Vector2D(v1.x + v2.x, v1.y + v2.y);
    }

    inline void operator +=(double s) { x +=s; y +=s; }
    inline void operator +=(const Vector2D& v2) { x +=v2.x; y +=v2.y; }
    inline void operator *=(double s) { x*=s; y*=s; }
    inline void operator *=(Vector2D const& s) { x*=s.x; y*=s.y; }
    inline void operator /=(double s) { x/=s; y/=s; }

    //                                                //private:
    double x, y;
    //double updateTime, accessTime;

    void Normalize()
    {
      if ((x==0)&&(y==0))
        return;
      double magnitude(len());
      x /= magnitude;
      y /= magnitude;
    }
};

class Vector3D {
  public:
    Vector3D(double _x,double _y,double _z):x(_x),y(_y),z(_z){}
    Vector3D(Vector2D const& v):x(v.x),y(v.y),z(0){}
    Vector3D():x(0),y(0),z(0){}
    inline void Set(double _x, double _y, double _z) { x=_x; y=_y; z=_z; }

    inline bool operator==(const Vector3D &rhs)const{return (fequal(x,rhs.x)&&fequal(y,rhs.y)&&fequal(z,rhs.z));}
    inline bool sameLoc(const Vector3D &rhs)const{return (fequal(x,rhs.x)&&fequal(y,rhs.y)&&fequal(z,rhs.z));}
    inline bool operator<(const Vector3D &rhs)const{return fequal(x,rhs.x)?(fequal(y,rhs.y)?fless(z,rhs.z):fless(y,rhs.y)):fless(x,rhs.x);}

    // Dot product
    inline double operator*(Vector3D const& other)const{return x * other.x + y * other.y + z * other.z;}
    // Negation
    inline Vector3D operator-(Vector3D const& other)const{return Vector3D(x-other.x,y-other.y,z-other.z);}
    inline void operator-=(Vector3D const& other){x-=other.x;y-=other.y;z-=other.z;}
    inline Vector3D operator-()const{return Vector3D(-x,-y,-z);}
    // Slope angle of this vector
    //inline double atan(){ return atan2(y,x); }
    // Square
    inline double sq(){ return (*this) * (*this); }
    inline double len(){ return sqrt(sq()); }
    inline Vector3D cross(Vector3D const& b)const{return Vector3D(y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x);}

    inline Vector3D operator/(const double num)const{return Vector3D(x/num, y/num, z/num);}

    inline Vector3D operator*(const double num)const{return Vector3D(x*num, y*num, z*num);}

    inline Vector3D operator+(const Vector3D& v2)const{return Vector3D(x+v2.x, y+v2.y, z+v2.z);}

    inline void operator+=(const Vector3D& v2){x+=v2.x; y+=v2.y;z+=v2.z;}
    inline bool operator<(const Vector3D& other){return x==other.x?y==other.y?other.z<z:y<other.y:x<other.x;}
    inline void operator *=(double s) { x*=s; y*=s; z*=s; }
    inline void operator /=(double s) { x/=s; y/=s; z/=s; }

    inline void Normalize(){
      if(x==0&&y==0&&z==0) return;
      double magnitude(len());
      x /= magnitude;
      y /= magnitude;
      z /= magnitude;
    }

    double x, y, z;
};

inline double distanceSquared(Vector3D const& A1, Vector3D const& B1){
  return (A1.x-B1.x)*(A1.x-B1.x) + (A1.y-B1.y)*(A1.y-B1.y) + (A1.z-B1.z)*(A1.z-B1.z);
}

inline double distanceOfPointToLine(Vector3D v, Vector3D w, Vector3D const& p){
  auto dw(w-v);
  const double l2(dw.sq());  // i.e. |w-v|^2 -  avoid a sqrt
  if (fequal(l2,0.0)) return (p*p + v*v);   // v == w case
  // Check whether the point lies on the line...
  auto dp(p-v);
  if(dp==dw || dp==-dw)return 0;
  return dw.cross(dp).len()/dw.len();
}

static bool collisionImminent(Vector3D A, Vector3D const& VA, double radiusA, double startTimeA, double endTimeA, Vector3D B, Vector3D const& VB, double radiusB, double startTimeB, double endTimeB){
  // assume time overlap
  if(fgreater(startTimeA,endTimeB)||fgreater(startTimeB,endTimeA)||fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return false;}

  if(A==B&&VA==VB&&(VA.x==0&&VA.y==0)){
    if(fgreater(startTimeA,endTimeB)||fgreater(startTimeB,endTimeA)){return false;}
    else{return true;}
  }

  if(fgreater(startTimeB,startTimeA)){
    // Move A to the same time instant as B
    A+=VA*(startTimeB-startTimeA);
    startTimeA=startTimeB;
  }else if(fless(startTimeB,startTimeA)){
    B+=VB*(startTimeA-startTimeB);
    startTimeB=startTimeA;
  }
  //if(fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return 0;}

  // Assume an open interval just at the edge of the agents...
  Vector3D w(B-A);
  double r(radiusA+radiusB-TOLERANCE); // Combined radius
  double c(w.sq()-r*r);
  if(c<0.0){return true;} // Agents are currently colliding

  // Use the quadratic formula to detect nearest collision (if any)
  Vector3D v(VA-VB);
  double a(v.sq());
  double b(w*v);

  double dscr(b*b-a*c);
  if(fleq(dscr,0)){ return false; }

  double ctime((b-sqrt(dscr))/a); // Collision time
  if(fless(ctime,0)){ return false; }

  // Collision will occur if collision time is before the end of the shortest segment
  return fleq(ctime,std::min(endTimeB,endTimeA)-startTimeA);
}

static void getFoci(double q,
                    double a,
                    double b,
                    double c,
                    double center_x,
                    double center_y,
                    double& fx1,
                    double& fy1,
                    double& fx2,
                    double& fy2){
  double s(.25*sqrt(fabs(q)*sqrt(b*b + (a-c)*(a-c)))); // Dist from center to foci
  double dscr(q*a-q*c);
  double dscr2(q*b);
  double theta(0);
  //if((dscr==0) && (dscr2==0)) θ=0
  if(dscr==0){
    if(dscr2>0) theta=.25*M_PI;
    else if(dscr2<0) theta=.75*M_PI;
  }else if(dscr>0){
    theta=.5*atan(b/(a-c));
    if(dscr2<0) theta+=M_PI;
  }else{
    theta=.5*atan(b/(a-c))+M_PI/2.0;
  }
  fx1=center_x+cos(theta)*s;
  fx2=center_x-cos(theta)*s;
  fy1=center_y+sin(theta)*s;
  fy2=center_y-sin(theta)*s;
}

static std::pair<double,double> getDelay(double fx1, // focal point 1 x coord
                       double fy1, // focal point 1 y coord
                       double fx2, // focal point 2 x coord
                       double fy2, // focal point 2 y coord
                       double s, // segment length
                       double r){ // radius
  double fx12(fx1*fx1);
  double fx22(fx2*fx2);
  double fy12(fy1*fy1);
  double fy22(fy2*fy2);
  double fxy1(fx1*fy1);
  double fx1x2(fx1*fx2);
  double fxy2(fx2*fy2);
  double fx1y2(fx1*fy2);
  double fy1x2(fy1*fx2);
  double r2=r*r;
  double s2=s*s;
  double c1(-4*fx12 + 8*fxy1 + 8*fx1x2 - 8*fx1y2 - 4*fy12 - 8*fy1x2 + 8*fy1*fy2 - 4*fx22 + 8*fxy2 - 4*fy22 + 8*r2);
  double num(-4*fx12*fx1 + 4*fx12*fy1 + 4*fx12*fx2 - 4*fx12*fy2 + 8*fx12*s - 4*fx1*fy12 - 8*fxy1*s + 4*fx1*fx22 - 16*fx1x2*s + 4*fx1*fy22 +
             8*fx1y2*s + 4*fx1*r2 + 4*fy12*fy1 + 4*fy12*fx2 - 4*fy12*fy2 - 4*fy1*fx22 + 8*fy1x2*s - 4*fy1*fy22 - 4*fy1*r2 - 4*fx22*fx2 +
             4*fx2*fxy2 + 8*fx22*s - 4*fx2*fy22 - 8*fxy2*s + 4*fx2*r2 + 4*fy22*fy2 - 4*fy2*r2 - 8*r2*s);
  double v1(4*fx12*fx1 - 4*fx12*fy1 - 4*fx12*fx2 + 4*fx12*fy2 - 8*fx12*s);
  double v2(sqrt(num*num - 4*c1*(-fx12*fx12 + 4*fx12*fx1*s - 2*fx12*fy12 + 2*fx12*fx22 - 4*fx12*fx2*s + 2*fx12*fy22 + 2*fx12*r2 -
                                 4*fx12*s2 + 4*fx1*fy12*s - 4*fx1*fx22*s + 8*fx1x2*s2 - 4*fx1*fy22*s - 4*fx1*r2*s - fy12*fy12 +
                                 2*fy12*fx22 - 4*fy12*fx2*s + 2*fy12*fy22 + 2*fy12*r2 - fx22*fx22 + 4*fx22*fx2*s - 2*fx22*fy22 +
                                 2*fx22*r2 - 4*fx22*s2 + 4*fx2*fy22*s - 4*fx2*r2*s - fy22*fy22 + 2*fy22*r2 - r2*r2 + 4*r2*s2)));
  double v3(4*fx1*fy12 + 8*fxy1*s - 4*fx1*fx22 + 16*fx1x2*s - 4*fx1*fy22 - 8*fx1y2*s - 4*fx1*r2 - 4*fy12*fy1 - 4*fy12*fx2 + 4*fy12*fy2 +
            4*fy1*fx22 - 8*fy1x2*s + 4*fy1*fy22 + 4*fy1*r2 + 4*fx22*fx2 - 4*fx2*fxy2 - 8*fx22*s + 4*fx2*fy22 + 8*fxy2*s - 4*fx2*r2 - 4* fy22*fy2 + 4*fy2*r2 + 8*r2*s);
  return {(v1 - v2 + v3)/(2*c1), (v1 + v2 + v3)/(2*c1)};
}

static std::pair<double,double>
getForbiddenIntervalIncremental(Vector3D const& A,
                     Vector3D const& VA,
                     double startTimeA,
                     double endTimeA,
                     double radiusA,
                     Vector3D const& B,
                     Vector3D const& VB,
                     double startTimeB,
                     double endTimeB,
                     double radiusB,
                     double delayStart,
                     double delayEnd,
                     double res=0.001){
  double dur(endTimeA-startTimeA);
  double durB(endTimeB-startTimeB);
    for(double i(delayStart+res); i<delayEnd; i+=res){
      if(collisionImminent(A,VA,radiusA,i,i+dur,B,VB,radiusB,0,durB)){
        delayStart=i-res;
        break;
      }
    }
    for(double i(delayEnd-res); i>delayStart; i-=res){
      if(collisionImminent(A,VA,radiusA,i,i+dur,B,VB,radiusB,0,durB)){
        delayEnd=i-res;
        break;
      }
    }
    //std::cout<<"USING INCREMENTAL\n";
    return {delayStart,delayEnd};
}

static std::pair<double,double>
getInterval(Vector2D A,
            Vector2D A2,
            double startTimeA,
            double endTimeA,
            Vector2D B,
            Vector2D B2,
            double startTimeB,
            double endTimeB)
{
    Vector2D VA(A2-A);
    VA.Normalize();
    Vector2D VB(B2-B);
    VB.Normalize();
    std::cout<<VA.x<<" "<<VA.y<<"\n";
    std::cout<<VB.x<<" "<<VB.y<<"\n";
    if(startTimeA > startTimeB)
    {
        B += VB*(startTimeA - startTimeB);
        startTimeB = startTimeA;
    }
    else if(startTimeB > startTimeA)
    {
        A += VA*(startTimeB - startTimeA);
        startTimeA = startTimeB;
    }
    double A_ = (VA - VB)*(VA - VB);
    double B_ = 2*(VA*VA - VB*VA);
    double C_ = VA*VA;
    double D_ = 2*(B*VB - B*VA + A*VB - A*VA);
    double E_ = -2*(B*VA + A*VA);
    double F_ = A_ - pow(1.0, 2);
    std::cout<<A_<<" "<<B_<<" "<<C_<<" "<<D_<<" "<<E_<<" "<<F_<<"\n";
    double center = (B_*D_ - 2*A_*E_)/(4*A_*C_ - B_*B_);
    double delay = sqrt(pow(2*B_*D_ - 4*A_*E_,2) +4*(4*A_*C_ - B_*B_)*(D_*D_ - 4*A_*F_))/(2*(4*A_*C_ - B_*B_));
    std::cout<<center<<" "<<delay<<"\n";
    return {center-delay, center+delay};

}

// General case for interval - does not consider the corner cases... (use getForbiddenInterval())
static std::pair<double,double>
getForbiddenIntervalGeneralCase(Vector3D const& A,
                     Vector3D const& A2,
                     Vector3D const& VA,
                     double dur, // Duration of action A
                     Vector3D const& B,
                     Vector3D const& B2,
                     Vector3D const& VB,
                     double durB,
                     Vector3D const& DAB, // A-B
                     Vector3D const& DVAB, // VA-VB
                     double r, // (rA+rB)
                     double res=0.001){
  double rsq(r*r);
  double AVA(A*VA);
  double BVA(B*VA);
  double AVB(A*VB);
  double BVB(B*VB);
 std::cout<<"INCREMENTAL\n";
  double a(DVAB*DVAB); // Dot product
  //b has the effect of rotating the conic
  double b(2.0*(VA*VA - VA*VB));
  double c(VA*VA);
  double d(2.0*(BVB - BVA + AVA - AVB));
  double e(-2.0*BVA+2.0*AVA);
  double f(DAB*DAB - rsq);

  // this conic is guaranteed to be an ellipse because both a and c are >= 0 (because they are squares)
  // the case of a or c being zero is covered by the above cases (opposing, parallel and waiting)
  double dscr(4.0*a*c-b*b);


  // Many of the following equations are from:
  // https://math.stackexchange.com/questions/616645/determining-the-major-minor-axes-of-an-ellipse-from-general-form
  double g(b*d-2.0*a*e);
  double center_y(g/dscr);
  g*=2;
  double delta(sqrt(g*g+4.0*dscr*(d*d-4*a*f))/(2*dscr));
  double delayStart(center_y-delta);
  double delayEnd(center_y+delta);
  //std::cout << "############### segs " << dur << "," << durB << "\n";
  //std::cout << "############### delay " << delayEnd << ", " << delayStart << "\n";

  double da1b(distanceOfPointToLine(B,B2,A));
  double da2b(distanceOfPointToLine(B,B2,A2));
  double db1a(distanceOfPointToLine(A,A2,B));
  double db2a(distanceOfPointToLine(A,A2,B2));
 /* if(da1b<r || da2b<r || db1a<r || db2a<r){
    //std::cout << da1b << " " << da2b << " " << db1a << " " << db2a << "\n";
    return getForbiddenIntervalIncremental(A,VA,0,dur,r/2.0,B,VB,0,durB,r/2.0,-delayEnd,-delayStart,res);
    //std::cout << "SPECIAL CASE\n";
    //for(double i(delayStart+res); i<delayEnd; i+=res){
      //if(collisionImminent(A,VA,r/2.0,0,dur,B,VB,r/2.0,i,i+durB)){
        //delayStart=i-res;
        //break;
      //}
    //}
    //for(double i(delayEnd-res); i>delayStart; i-=res){
      //if(collisionImminent(A,VA,r/2.0,0,dur,B,VB,r/2.0,i,i+durB)){
        //delayEnd=i-res;
        //break;
      //}
    //}
  }else*/ if(distanceSquared(A,B2)<rsq && da2b>r){
    delayStart=-durB;
  }else{
    // Check the collision time for delayEnd.
    // If it occurs after the common interval, we must truncate it to the delay associated with
    // the end of the common interval (find the y-coordinate on the ellipse for x=tEnd)
    double collisionTimeApex((-b*delayEnd-d)/(2*a));
    double collisionTimeBase((-b*delayStart-d)/(2*a));
    // Determine if segments end before being able to collide
    double h(b*e - 2*c*d);
    double center_x(h/dscr);
    h*=2;
    double leftx(center_x-sqrt(h*h+4.0*dscr*(e*e-4*c*f))/(2*dscr));
    //double rightx(center_x+sqrt(h*h+4.0*dscr*(e*e-4*c*f))/(2*dscr));
    //double lefty((-b*leftx-e)/(2*c));
    //std::cout << "############### apex " << collisionTimeApex << ", " << collisionTimeBase << "\n";
    //std::cout << "############### left/right " << leftx << ", " << rightx << "\n";
    //std::cout << "len " << (dur+delayStart) << "<" << collisionTimeApex << "=" << ((dur+delayStart)<collisionTimeApex) << ", " << (dur+delayEnd) << "<" << collisionTimeApex << "=" << ((dur+delayEnd)<collisionTimeApex) << "\n";
    // len A > len B and A1 is near B2 and orthogonal distance from A2 to B is large, then the forbidden interval ends when agent B disappears (len B)
    if(!fequal(leftx,collisionTimeApex) && (collisionTimeApex+delayEnd>dur || collisionTimeBase>durB)){
      double q(64*(f*dscr-a*e*e+b*d*e-c*d*d)/(dscr*dscr));
      double fx1;
      double fy1;
      double fx2;
      double fy2;
      getFoci(q,a,b,c,center_x,center_y,fx1,fy1,fx2,fy2);
      double dx1(collisionTimeApex-fx1);
      double dx2(collisionTimeApex-fx2);
      double dy1(delayEnd-fy1);
      double dy2(delayEnd-fy2);
      double focrad(sqrt(dx1*dx1 + dy1*dy1) + sqrt(dx2*dx2 + dy2*dy2));
      //std::cout << "Start was " << delayStart << "\n";
      //std::cout << "End was " << delayEnd << "\n";
      if(collisionTimeApex+delayEnd>dur){
        auto delays(getDelay(fx1,fy1,fx2,fy2,dur,focrad));
        //std::cout << "begin: " << delays.first << "," << delays.second << "\n";
        delayEnd=delays.second;
      }
      if(collisionTimeApex>(durB-dur)){
        auto delays(getDelay(fx1,fy1,fx2,fy2,(durB-dur)+delayEnd,focrad));
        //std::cout << "diff: " << delays.first << "," << delays.second << "\n";
        if(!isnan(delays.first)){
          delayEnd=delays.first;
        }
      }
      if(collisionTimeBase>durB){
        // Does applying the delay make the collision time end too early?
        auto delays(getDelay(fx1,fy1,fx2,fy2,durB+delayStart,focrad));
        //std::cout << "end: " << delays.first << "," << delays.second << "\n";
        if(!isnan(delays.first)){
          delayStart=delays.first;
        }else{
          auto delays(getDelay(fx1,fy1,fx2,fy2,durB+delayEnd,focrad));
          //std::cout << "end2: " << delays.first << "," << delays.second << "\n";
          if(!isnan(delays.first))
            delayStart=delays.first;
        }
      }
      //std::cout << "Start now " << delayStart << "\n";
      //std::cout << "End now " << delayEnd << "\n";
    }
  }
  return {-delayEnd,-delayStart};
}

static std::pair<double,double>
getForbiddenInterval(Vector3D A,
                     Vector3D A2,
                     double startTimeA,
                     double endTimeA,
                     double radiusA,
                     Vector3D B,
                     Vector3D B2,
                     double startTimeB,
                     double endTimeB,
                     double radiusB,
                     double res=0.001){
  Vector3D VA(A2-A);
  Vector3D VB(B2-B);
  VA.Normalize();
  VB.Normalize();
  /*if(startTimeB > startTimeA)
  {
      A += VA*(startTimeB-startTimeA);
      startTimeA = startTimeB;
  }
  else if(startTimeB < startTimeA)*/
  {
      B += VB*(startTimeA - startTimeB);
      startTimeB = startTimeA;
  }
  //std::cout<<startTimeA<<"\n";
  //std::cout<<A.x<<" "<<A.y<<"   "<<B.x<<" "<<B.y<<"\n";
  //std::cout<<A2.x<<" "<<A2.y<<"   "<<B2.x<<" "<<B2.y<<"\n";
  double dur(endTimeA-startTimeA);
  Vector3D DAB(B-A);
  // Assume unit speed...
  double r(radiusA+radiusB);
  double rsq(r*r);
  Vector3D DVAB(VB-VA);

  // Is one agent waiting?
  if(VA.x==0 && VA.y==0){
    if(VB.x==0 && VB.y==0){ // Both waiting
      if(DAB.sq()<rsq){return {startTimeB-dur,endTimeB}; //Overlapping and waiting
      }else{
        return {std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};} // Not overlapping and waiting
    }
    // Compute the interval until agent B passes over agent A
    double d(distanceOfPointToLine(B,B2,A));
    d*=d;
    if(d<rsq){
      double durB(endTimeB-startTimeB);
      double v(sqrt(rsq-d)); // translational distance between centers at start of impact
      double dist(sqrt(DAB.sq()-d)); // distance traveled by either agent
      // Backoff=time of collision - total time of intersection
      //std::cout << "v " << v << " dur " << dur << "\n";
      //std::cout << "i " << startTimeB+(dist-v-dur) << "," << startTimeB+(dist+v) << "\n";
      //return {std::max(startTimeB-dur,startTimeB+(dist-v-dur)),std::min(startTimeB+(dist+v),endTimeB)}; // Distance from line is close enough to crash.
      return getForbiddenIntervalIncremental(A,VA,0,dur,radiusA,B,VB,0,durB,radiusB,std::max(startTimeB-dur,startTimeB+(dist-v-dur)),std::min(startTimeB+(dist+v),endTimeB),res);
    }else{ return {std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};} // never conflicting
  }else if(VB.x==0 && VB.y==0){
    double d(distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      //double v(sqrt(rsq-d)); // translational distance between centers at start of impact
      //double dist(sqrt(DAB.sq()-d)); // distance traveled by either agent
      double durB(endTimeB-startTimeB);
      return getForbiddenIntervalIncremental(A,VA,0,dur,radiusA,B,VB,0,durB,radiusB,startTimeA-std::max(dur,durB),endTimeB,res);
      //std::cout << v << " " << dist << "\n";
      //std::cout << (startTimeB-durB) << " " << (startTimeB+(-durB+dist+v)) << "," << (startTimeB+(dist+v)) << " " << (endTimeB) << "\n";
      //std::cout << (startTimeA-durB) << " " << (startTimeA+(dist-v)) << "," << (startTimeA+(dist+v)) << " " << (endTimeB) << "\n";
      //return {std::max(startTimeA-durB,startTimeA+(dist-v)),std::max(startTimeA+(dist+v),endTimeB)}; // Distance from line is close enough to crash.
    }else{ return {std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};} // never conflicting
  }

  // Are they parallel?
  if(fequal(DVAB.x,0.0) && fequal(DVAB.y,0.0)){
    // If they are currently overlapping, they will do so forever, otherwise, they never will
    double d(distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // translational orthogonal distance between centers at start of impact
      double ds(DAB.sq()); // Sq. distance between starts
      double dc(sqrt(ds-d)); // Distance between starts along trajectory line
      // Determine if A is "in front of" B
      Vector3D DA(A2-A);
      double costheta(DAB*DA/(DAB.len()*DA.len()));
      if(costheta>0){
        return {std::max(startTimeB-dc-v,-endTimeA),std::min(startTimeB-dc+v,endTimeB)};
      }else if(costheta<0){
        return {std::max(startTimeB+dc-v,-endTimeA),std::min(startTimeB+dc+v,endTimeB)};
      }
      return {startTimeB-v,std::min(startTimeB+v,endTimeB)}; // Distance between lines is close enough to crash.
    }else{ return {std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};} // Parallel, never conflicting
  }
  // Are they opposing?
  if(VA==-VB){
    double d(distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // translational distance between centers at start of impact

      auto ES(B-A2);
      auto SE(B2-A);

      double dep(sqrt(ES.sq()-d)); // distance required for A to be orthogonal with start of B
      double dp(sqrt(SE.sq()-d)); // distance required for A to be orthogonal with end of B

      double start(0.0f);
      double end(0.0f);

      Vector3D DA(A2-A);
      if(SE.len()<=r){
        end=endTimeB;
      }else{
        double costheta(SE*DA/(SE.len()*DA.len()));
        if(costheta>=0){
          end=endTimeB-dp+v;
        }else{
          end=endTimeB-dp+v;
        }
      }

      // Determine if A2 is "behind" B
      if(ES.len()<=r){
        start=startTimeB-dur;
      }else{
        double costheta(ES*DA/(ES.len()*DA.len()));
        if(costheta>0){ // Acute angle
          start=startTimeB-dur+dep-v;
        }else{
          start=startTimeB-dur+dep-v;
        }
      }
      return {start,end};
    }else{ return {std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};} // Parallel, never conflicting
  }


  // General case
  double durB(endTimeB-startTimeB);
  if(durB<dur){
    std::cout << "SWAPPED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
    auto intvl(getForbiddenIntervalGeneralCase(B,B2,VB,durB,A,A2,VA,dur,DAB,DVAB,r));
    return {-intvl.second,-intvl.first};
  }
  return getForbiddenIntervalGeneralCase(A,A2,VA,dur,B,B2,VB,durB,DAB,DVAB,r);
}

#endif // COLLISION_INTERVAL_H
