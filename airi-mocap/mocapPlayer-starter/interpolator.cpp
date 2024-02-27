#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include "vector.h"
#include <glm/gtc/type_ptr.hpp> // For glm::make_mat4
#include <glm/glm.hpp>
#include <cmath>

double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

vector Interpolator::Lerp(double t, vector& vStart, vector& vEnd)
{
    return vStart * (1.0 - t) + vEnd * t;
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 
  
  // N: number of skipped frames
  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this
    double radX = degreesToRadians(angles[0]);
    glm::dmat3 rotX = glm::dmat3(1.0, 0.0, 0.0,
        0.0, cos(radX), sin(radX),
        0.0, -sin(radX), cos(radX));
    
    double radY = degreesToRadians(angles[1]);
    glm::dmat3 rotY = glm::dmat3(cos(radY), 0.0, -sin(radY),
        0.0, 1.0, 0.0,
        sin(radY), 0.0, cos(radY));
    
    double radZ = degreesToRadians(angles[2]);
    glm::dmat3 rotZ = glm::dmat3(cos(radZ), sin(radZ), 0.0,
        -sin(radZ), cos(radZ), 0.0,
        0.0, 0.0, 1.0);

    glm::dmat3 rot = (rotZ * rotY * rotX);
    std::memcpy(R, &rot, sizeof(rot));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double>& q)
{
    // students should implement this
    double R[9];
    Euler2Rotation(angles, R); // angles -> rotation matrix
    q = Quaternion<double>::Matrix2Quaternion(R); // rotation matrix -> quaternion
}

void Interpolator::Quaternion2Euler(Quaternion<double>& q, double angles[3])
{
    // students should implement this
    double R[9];
    q.Quaternion2Matrix(R); // quaternion -> rotation matrix
    Rotation2Euler(R, angles); // rotation matrix -> euler angles
}

// quaternion interpolation
Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
    double theta = acos(qStart.Gets() * qEnd_.Gets() 
        + qStart.Getx() * qEnd_.Getx()
        + qStart.Gety() * qEnd_.Gety() 
        + qStart.Getz() * qEnd_.Getz()); // in radian
  Quaternion<double> result = qStart * (sin(theta * (1.0 - t)) / sin(theta)) + qEnd_ * (sin(theta * t) / sin(theta));
  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result = 2 * (p.Gets() * q.Gets()
      + p.Getx() * q.Getx()
      + p.Gety() * q.Gety()
      + p.Getz() * q.Getz()) * q - p;
  return result;
}

// Bezier spline evaluation
vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector q0 = Lerp(t, p0, p1);
  vector q1 = Lerp(t, p1, p2);
  vector q2 = Lerp(t, p2, p3);
  vector r0 = Lerp(t, q0, q1);
  vector r1 = Lerp(t, q1, q2);
  return Lerp(t, r0, r1);
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

// interpolation routines
void Interpolator::BezierInterpolationEuler(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    // students should implement this
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    int startKeyframe = 0, hasPrev = 0, hasNext = 0;

    while (startKeyframe + N + 1 < inputLength)
    {
        hasPrev = 0;
        hasNext = 0;
        int prevKeyframe = startKeyframe - N - 1;
        int endKeyframe = startKeyframe + N + 1;
        int nextKeyframe = endKeyframe + N + 1;
        startKeyframe = endKeyframe;

        vector p0, p1, p2, p3; // points
        vector a1, b2; // control points

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        p1 = startPosture->root_pos;
        p2 = endPosture->root_pos;

        // if there's a prev frame
        Posture* prevPosture;
        if (prevKeyframe > 0)
        {
            prevPosture = pInputMotion->GetPosture(prevKeyframe);
            p0 = prevPosture->root_pos;
            hasPrev = 1;
        }

        // if there's a next frame
        Posture* nextPosture;
        if (nextKeyframe < inputLength)
        {
            nextPosture = pInputMotion->GetPosture(nextKeyframe);
            p3 = nextPosture->root_pos;
            hasNext = 1;
        }

        // find control points
        if (!hasPrev && hasNext)
        {
            // special case a1
            a1 = (1.0 / 3.0, p1, Lerp(2.0, p3, p2)); 
            // find b2
            vector a2_ = Lerp(0.5, Lerp(2.0, p1, p2), p3);
            b2 = Lerp(-1.0 / 3.0, p2, a2_);
        }
        else if (!hasNext && hasPrev)
        {
            // find a1
            vector a1_ = Lerp(0.5, Lerp(2.0, p0, p1), p2);
            a1 = Lerp(1.0 / 3.0, p1, a1_);
            // special case b2
            b2 = p2 * (2.0 / 3.0) + (p1 * 2.0 - p0) * (1.0 / 3.0);
        }
        else if (hasNext && hasPrev)
        {
            // find a1
            vector a1_ = Lerp(0.5, Lerp(2.0, p0, p1), p2);
            a1 = Lerp(1.0 / 3.0, p1, a1_);
            // find b2
            vector a2_ = Lerp(0.5, Lerp(2.0, p1, p2), p3);
            b2 = Lerp(-1.0 / 3.0, p2, a2_);
        }

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            double t = 1.0 * frame / (N + 1.0); // [0, 1]

            // interpolate root position (Bezier)
            Posture interpolatedPosture;
            interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, p2, a1, b2);

            // interpolate bone rotations (Bezier)
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {

                interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1 - t) + endPosture->bone_rotation[bone] * t;
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    // students should implement this
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    int startKeyframe = 0;

    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;
        startKeyframe = endKeyframe;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);
    }
}

void Interpolator::BezierInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    // students should implement this
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    int startKeyframe = 0;

    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;
        startKeyframe = endKeyframe;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);
    }
}
