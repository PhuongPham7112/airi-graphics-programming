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
#include <iostream>
#include <chrono>

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
  
  // Start measuring time
  auto start = std::chrono::high_resolution_clock::now();
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

  // Stop measuring time
  auto stop = std::chrono::high_resolution_clock::now();

  // Calculate the duration in milliseconds
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

  // Output the execution time
  std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;
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

    glm::dmat3 rot = glm::transpose(rotZ * rotY * rotX);
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
    q.Normalize(); // WARNING: modifying q
    q.Quaternion2Matrix(R); // quaternion -> rotation matrix
    Rotation2Euler(R, angles); // rotation matrix -> euler angles
}

// quaternion interpolation
Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this. WARNING: need more checking edge cases
    Quaternion<double> result;
    double dotProduct = qStart.Gets() * qEnd_.Gets()
        + qStart.Getx() * qEnd_.Getx()
        + qStart.Gety() * qEnd_.Gety()
        + qStart.Getz() * qEnd_.Getz();

    // opposite
    if (dotProduct < 0.0)
    {
        qEnd_ = qEnd_ * -1.0;
        dotProduct = -dotProduct;
    }
    // identical
    else if (dotProduct > 0.0 && std::abs(dotProduct) >= 1.0)
    {
        return qStart;
    }
    double theta = acos(dotProduct); // in radian
    double sintheta = sqrt(1.0 - dotProduct * dotProduct);
    // division by zero case
    if (std::abs(sintheta) < 0.001)
    {
        result = qStart * 0.5 + qEnd_ * 0.5;
    }
    result = qStart * (sin(theta * (1.0 - t)) / sintheta) + qEnd_ * (sin(theta * t) / sintheta);
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
  Quaternion<double> q0 = Slerp(t, p0, p1);
  Quaternion<double> q1 = Slerp(t, p1, p2);
  Quaternion<double> q2 = Slerp(t, p2, p3);
  Quaternion<double> r0 = Slerp(t, q0, q1); 
  Quaternion<double> r1 = Slerp(t, q1, q2);
  return Slerp(t, r0, r1);
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
      double t = 1.0 * frame / (N + 1.0);

      // interpolate root position
      interpolatedPosture.root_pos = Lerp(t, startPosture->root_pos, endPosture->root_pos);

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
        int prevKeyframe = startKeyframe - (N + 1);
        int endKeyframe = startKeyframe + N + 1;
        int nextKeyframe = endKeyframe + N + 1;

        // prev, start, end, next
        vector p0, p1, p2, p3;
        // control points
        vector a1, b2; 

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);
        p1 = startPosture->root_pos;
        p2 = endPosture->root_pos;

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            double t = 1.0 * frame / (N + 1.0); // [0, 1]

             // if there's a prev frame
            Posture* prevPosture;
            if (prevKeyframe >= 0)
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
            if (!hasPrev)
            {
                // special case a1
                a1 = Lerp(1.0 / 3.0, p1, Lerp(2.0, p3, p2));
                // find b2
                vector a2_ = Lerp(0.5, Lerp(2.0, p1, p2), p3);
                b2 = Lerp(-1.0 / 3.0, p2, a2_);
            }
            else if (!hasNext)
            {
                // find a1
                vector a1_ = Lerp(0.5, Lerp(2.0, p0, p1), p2);
                a1 = Lerp(1.0 / 3.0, p1, a1_);
                // special case b2
                b2 = Lerp(1.0 / 3.0, p2, Lerp(2.0, p0, p1));
            }
            else
            {
                // find a1
                vector a1_ = Lerp(0.5, Lerp(2.0, p0, p1), p2);
                a1 = Lerp(1.0 / 3.0, p1, a1_);
                // find b2
                vector a2_ = Lerp(0.5, Lerp(2.0, p1, p2), p3);
                b2 = Lerp(-1.0 / 3.0, p2, a2_);
            }

            // interpolate root position (Bezier)
            Posture interpolatedPosture;
            interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a1, b2, p2);

            // interpolate bone rotations (Bezier)
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                vector r0, r1, r2, r3; // bone rotation

                r1 = startPosture->bone_rotation[bone];
                r2 = endPosture->bone_rotation[bone];

                if (hasPrev)
                {
                    r0 = prevPosture->bone_rotation[bone];
                }

                if (hasNext)
                {
                    r3 = nextPosture->bone_rotation[bone];
                }

                // find control points
                if (!hasPrev)
                {
                    // special case a1
                    a1 = Lerp(1.0 / 3.0, r1, Lerp(2.0, r3, r2));
                    // find b2
                    vector a2_ = Lerp(0.5, Lerp(2.0, r1, r2), r3);
                    b2 = Lerp(-1.0 / 3.0, r2, a2_);
                }
                else if (!hasNext)
                {
                    // find a1
                    vector a1_ = Lerp(0.5, Lerp(2.0, r0, r1), r2);
                    a1 = Lerp(1.0 / 3.0, r1, a1_);
                    // special case b2
                    b2 = Lerp(1.0 / 3.0, r2, Lerp(2.0, r0, r1));
                }
                else
                {
                    // find a1
                    vector a1_ = Lerp(0.5, Lerp(2.0, r0, r1), r2);
                    a1 = Lerp(1.0 / 3.0, r1, a1_);
                    // find b2
                    vector a2_ = Lerp(0.5, Lerp(2.0, r1, r2), r3);
                    b2 = Lerp(-1.0 / 3.0, r2, a2_);
                }

                interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, r1, a1, b2, r2);
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

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            double t = 1.0 * frame / (N + 1.0);

            // interpolate root position
            Posture interpolatedPosture;
            interpolatedPosture.root_pos = Lerp(t, startPosture->root_pos, endPosture->root_pos);

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> qStart;
                Euler2Quaternion(startPosture->bone_rotation[bone].p, qStart);

                Quaternion<double> qEnd;
                Euler2Quaternion(endPosture->bone_rotation[bone].p, qEnd);

                Quaternion<double> qInterpolated = Slerp(t, qStart, qEnd);
                Quaternion2Euler(qInterpolated, interpolatedPosture.bone_rotation[bone].p);
            }
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        startKeyframe = endKeyframe;
    }
    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    // students should implement this
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    int startKeyframe = 0, hasPrev = 0, hasNext = 0;

    while (startKeyframe + N + 1 < inputLength)
    {
        hasPrev = 0;
        hasNext = 0;
        int prevKeyframe = startKeyframe - (N + 1);
        int endKeyframe = startKeyframe + N + 1;
        int nextKeyframe = endKeyframe + N + 1;

        // prev, start, end, next quat
        Quaternion<double> q0, q1, q2, q3;
        // control points for rotation
        Quaternion<double> qa1, qb2;
        // prev, start, end, next root pos
        vector p0, p1, p2, p3;
        // control points for root pos
        vector a1, b2;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);
        
        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        p1 = startPosture->root_pos;
        p2 = endPosture->root_pos;

        // if there's a prev frame
        Posture* prevPosture;
        if (prevKeyframe >= 0)
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

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            double t = 1.0 * frame / (N + 1.0); // [0, 1]

            // find control points
            if (!hasPrev)
            {
                // special case a1
                a1 = Lerp(1.0 / 3.0, p1, Lerp(2.0, p3, p2));
                // find b2
                vector a2_ = Lerp(0.5, Lerp(2.0, p1, p2), p3);
                b2 = Lerp(-1.0 / 3.0, p2, a2_);
            }
            else if (!hasNext)
            {
                // find a1
                vector a1_ = Lerp(0.5, Lerp(2.0, p0, p1), p2);
                a1 = Lerp(1.0 / 3.0, p1, a1_);
                // special case b2
                b2 = Lerp(1.0 / 3.0, p2, Lerp(2.0, p0, p1));
            }
            else
            {
                // find a1
                vector a1_ = Lerp(0.5, Lerp(2.0, p0, p1), p2);
                a1 = Lerp(1.0 / 3.0, p1, a1_);
                // find b2
                vector a2_ = Lerp(0.5, Lerp(2.0, p1, p2), p3);
                b2 = Lerp(-1.0 / 3.0, p2, a2_);
            }

            // interpolate root position (Bezier)
            Posture interpolatedPosture;
            interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a1, b2, p2);

            // interpolate bone rotations (Bezier)
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Euler2Quaternion(startPosture->bone_rotation[bone].p, q1);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, q2);

                if (hasPrev)
                {
                    Euler2Quaternion(prevPosture->bone_rotation[bone].p, q0);
                }

                if (hasNext)
                {
                    Euler2Quaternion(nextPosture->bone_rotation[bone].p, q3);
                }

                // find control points
                if (!hasPrev)
                {
                    // special case a1
                    qa1 = Slerp(1.0 / 3.0, q1, Double(q3, q2));
                    // find b2
                    Quaternion<double> qa2_ = Slerp(0.5, Double(q1, q2), q3);
                    qb2 = Slerp(-1.0 / 3.0, q2, qa2_);
                }
                else if (!hasNext)
                {
                    // find a1
                    Quaternion<double> qa1_ = Slerp(0.5, Double(q0, q1), q2);
                    qa1 = Slerp(1.0 / 3.0, q1, qa1_);
                    // special case b2
                    qb2 = Slerp(1.0 / 3.0, q2, Double(q0, q1));
                }
                else
                {
                    // find a1
                    Quaternion<double> qa1_ = Slerp(0.5, Double(q0, q1), q2);
                    qa1 = Slerp(1.0 / 3.0, q1, qa1_);
                    // find b2
                    Quaternion<double> qa2_ = Slerp(0.5, Double(q1, q2), q3);
                    qb2 = Slerp(-1.0 / 3.0, q2, qa2_);
                }
                Quaternion2Euler(DeCasteljauQuaternion(t, q1, qa1, qb2, q2), interpolatedPosture.bone_rotation[bone].p);
            }
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        startKeyframe = endKeyframe;
    }
    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}
