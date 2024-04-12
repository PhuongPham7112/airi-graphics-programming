#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

namespace
{

// Converts degrees to radians.
template<typename real>
inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

template<typename real>
Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
{
  Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
  Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
  Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

  switch(order)
  {
    case RotateOrder::XYZ:
      return RZ * RY * RX;
    case RotateOrder::YZX:
      return RX * RZ * RY;
    case RotateOrder::ZXY:
      return RY * RX * RZ;
    case RotateOrder::XZY:
      return RY * RZ * RX;
    case RotateOrder::YXZ:
      return RZ * RX * RY;
    case RotateOrder::ZYX:
      return RX * RY * RZ;
  }
  assert(0);
}

// Performs forward kinematics, using the provided "fk" class.
// This is the function whose Jacobian matrix will be computed using adolc.
// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
//   IKJointIDs is an array of integers of length "numIKJoints"
// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
template<typename real>
void forwardKinematicsFunction(
    int numIKJoints, const int * IKJointIDs, const FK & fk,
    const std::vector<real> & eulerAngles, std::vector<real> & handlePositions)
{
  // Students should implement this.
  // The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
  // The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
  // Then, implement the same algorithm into this function. To do so,
  // you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
  // Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .

    // constructing the order from given ids
    std::vector<int> orderedIDs(numIKJoints);
    for (int i = 0; i < numIKJoints; i++)
    {
        int id = IKJointIDs[i];
        int order = fk.getJointUpdateOrder(id);
        orderedIDs[order] = id;
    }

    // The usual FK algorithms
    real angles[3];
    vector<Mat3<real>> globalRotTransforms(numIKJoints);
    vector<Vec3<real>> globalTransTransforms(numIKJoints);
    for (int i : orderedIDs)
    {
        // local transformation
        angles[0] = eulerAngles[i * 3];
        angles[1] = eulerAngles[i * 3 + 1];
        angles[2] = eulerAngles[i * 3 + 2];
        Mat3<real> localRotation = Euler2Rotation(angles, fk.getJointRotateOrder(i));

        // joint orient
        angles[0] = fk.getJointOrient(i)[0];
        angles[1] = fk.getJointOrient(i)[1];
        angles[2] = fk.getJointOrient(i)[2];
        Mat3<real> jointOrient = Euler2Rotation(angles, fk.getJointRotateOrder(i));

        // calculate local trans
        Mat3<real> localRot = jointOrient * localRotation;
        Vec3<real> localTrans = Vec3<real>(fk.getJointRestTranslation(i)[0], fk.getJointRestTranslation(i)[1], fk.getJointRestTranslation(i)[2]);

        int parentID = fk.getJointParent(i);
        if (parentID == -1)
        {
            globalRotTransforms[i] = localRot;
            globalTransTransforms[i] = localTrans;
        }
        else
        {
            multiplyAffineTransform4ds(globalRotTransforms[parentID], globalTransTransforms[parentID],
                localRot, localTrans,
                globalRotTransforms[i], globalTransTransforms[i]);
        }

        // todo: https://piazza.com/class/lr4b78cr4gb4om/post/109
        Vec3<real> handle = globalRotTransforms[i] * Vec3<real>(0, 0, 0) + globalTransTransforms[i];
        handlePositions[i * 3] = handle[0];
        handlePositions[i * 3 + 1] = handle[1];
        handlePositions[i * 3 + 2] = handle[2];
    }
  // It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
  // so that code is only written once. We considered this; but it is actually not easily doable.
  // If you find a good approach, feel free to document it in the README file, for extra credit.
}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID)
{
  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->fk = inputFK;
  this->adolc_tagID = adolc_tagID;

  FKInputDim = fk->getNumJoints() * 3;
  FKOutputDim = numIKJoints * 3;

  train_adolc();
}

void IK::train_adolc()
{
  // Students should implement this.
  // Here, you should setup adol_c:
  //   Define adol_c inputs and outputs. 
  //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
  //   This will later make it possible for you to compute the gradient of this function in IK::doIK
  //   (in other words, compute the "Jacobian matrix" J).
  // See ADOLCExample.cpp .
    int n = this->FKInputDim; // input dimension n
    int m = this->FKOutputDim; // output dimension m
    
    trace_on(this->adolc_tagID); // start tracking computation with ADOL-C
    
    vector<adouble> x(n); // define the input of the function f
    for (int i = 0; i < n; i++)
        x[i] <<= 0.0; // The <<= syntax tells ADOL-C that these are the input variables.

    vector<adouble> y(m); // define the output of the function f
    
    // The computation of f goes here:
    forwardKinematicsFunction(this->numIKJoints, this->IKJointIDs, *this->fk, x, y);
    
    vector<double> output(m);
    for (int i = 0; i < m; i++)
        y[i] >>= output[i]; // Use >>= to tell ADOL-C that y[i] are the output variables

    // Finally, call trace_off to stop recording the function f.
    trace_off(); // ADOL-C tracking finished
}

void IK::doIK(const Vec3d * targetHandlePositions, Vec3d * jointEulerAngles)
{
  // You may find the following helpful:
  int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!

  // Students should implement this.
  // Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
  // Specifically, use ::function, and ::jacobian .
  // See ADOLCExample.cpp .
  //
  // Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
  // Note that at entry, "jointEulerAngles" contains the input Euler angles. 
  // Upon exit, jointEulerAngles should contain the new Euler angles.
  double* anglesArray = new double[FKInputDim];
  double* handlesArray = new double[FKOutputDim];
  jointEulerAngles->convertToArray(anglesArray);
  targetHandlePositions->convertToArray(handlesArray);
  ::function(adolc_tagID, FKInputDim, FKOutputDim, anglesArray, handlesArray);
}

