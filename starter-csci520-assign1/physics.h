/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#ifndef _PHYSICS_H_
#define _PHYSICS_H_

void computeAcceleration(struct world* jello, struct point a[8][8][8]);
point HookLaw(double kHook, double restLength, point A, point B);
point Damping(double kDamp, point A, point B, point velA, point velB);
double distance(point A, point B);
void calculateExternalForce(world* jello, int x, int y, int z, point& a);

// perform one step of Euler and Runge-Kutta-4th-order integrators
// updates the jello structure accordingly
void Euler(struct world * jello);
void RK4(struct world * jello);

#endif

