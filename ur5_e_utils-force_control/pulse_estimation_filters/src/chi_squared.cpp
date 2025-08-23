/**
 *
 * \file chi_squared.cpp
 *
 * \brief Implementation of the Chi-Square Distribution & Incomplete Gamma Function in C
 *
 * \details Based on the formulas found here:
 *
 * Wikipedia - Incomplete Gamma Function -> Evaluation formulae -> Connection with Kummer's confluent hypergeometric
 * function
 * http://en.wikipedia.org/wiki/Regularized_Gamma_function#Connection_with_Kummer.27s_confluent_hypergeometric_function
 *
 * Wikipedia - Chi-squared Distribution -> Cumulative distribution function
 * http://en.wikipedia.org/wiki/Chi-squared_distribution#Cumulative_distribution_function
 *
 * These functions are placed in the Public Domain, and may be used by anyone, anywhere, for any reason, absolutely free
 * of charge.
 *
 * \author Jacob Wells
 * July 31, 2012
 *
 */
#include <math.h>
#include <stdio.h>

#include "pulse_estimation_filters/chi_squared.hpp"

/**
 *
 * \brief A is the level of accuracy you wish to calculate. Spouge's Approximation is slightly tricky, as you can only
 * reach the desired level of precision, if you have EXTRA precision available so that it can build up to the desired
 * level.
 *
 * \details If you're using double (64 bit wide datatype), you will need to set A to 11, as well as remember to change
 * the math functions to the regular (i.e. pow() instead of powl())
 *
 * double A = 11
 * long double A = 15
 *
 * !!! IF YOU GO OVER OR UNDER THESE VALUES YOU WILL LOSE PRECISION !!!
 *
 */
#define A 15  // 15

double chisqr(int Dof, double Cv)
{
  // printf("Dof:  %i\n", Dof);
  // printf("Cv:  %f\n", Cv);
  if(Cv < 0 || Dof < 1)
  {
    return 0.0;
  }
  double K = ((double)Dof) * 0.5;
  double X = Cv * 0.5;
  if(Dof == 2)
  {
    return exp(-1.0 * X);
  }
  long double PValue, Gam;
  long double ln_PV;
  ln_PV = log_igf(K, X);

  Gam = approx_gamma(K);
  // Gam = lgammal(K);
  // Gam = log_gamma(K);

  ln_PV -= Gam;
  PValue = 1.0 - expl(ln_PV);

  return (double)PValue;
}

/**
 *
 * \brief Returns the Natural Logarithm of the Incomplete Gamma Function
 *
 * \details I converted the ChiSqr to work with Logarithms, and only calculate the finised Value right at the end. This
 * allows us much more accurate calculations.  One result of this is that I had to increase the Number of Iterations
 * from 200 to 1000.  Feel free to play around with this if you like, but this is the only way I've gotten to work.
 * Also, to make the code easier to work it, I separated out the main loop.
 *
 */
static long double log_igf(long double S, long double Z)
{
  if(Z < 0.0)
  {
    return 0.0;
  }

  long double Sc, K;

  Sc = (logl(Z) * S) - Z - logl(S);

  K = KM(S, Z);

  return logl(K) + Sc;
}

static long double KM(long double S, long double Z)
{
  long double Sum = 1.0;
  long double Nom = 1.0;
  long double Denom = 1.0;

  for(int I = 0; I < 1000; I++)  // Loops for 1000 iterations
  {
    Nom *= Z;
    S++;
    Denom *= S;
    Sum += (Nom / Denom);
  }

  return Sum;
}

/**
 *
 * \brief Incomplete Gamma Function
 *
 * \details No longer need as I'm now using the log_igf(), but I'll leave this here anyway.
 *
 */
static double igf(double S, double Z)
{
  if(Z < 0.0)
  {
    return 0.0;
  }
  long double Sc = (1.0 / S);
  Sc *= powl(Z, S);
  Sc *= expl(-Z);

  long double Sum = 1.0;
  long double Nom = 1.0;
  long double Denom = 1.0;

  for(int I = 0; I < 200; I++)  // 200
  {
    Nom *= Z;
    S++;
    Denom *= S;
    Sum += (Nom / Denom);
  }

  return Sum * Sc;
}

/**
 *
 * \brief Implementation of the Gamma function using Spouge's Approximation in C.
 *
 */
double gamma(double N)
{
  /*
   * The constant SQRT2PI is defined as sqrt(2.0 * PI). For speed the constant is already defined in decimal form.
   * However, if you wish to ensure that you achieve maximum precision on your own machine, you can calculate it
   * yourself using (sqrt(atan(1.0) * 8.0))
   */

  // const long double SQRT2PI = sqrtl(atanl(1.0) * 8.0);
  const long double SQRT2PI = 2.5066282746310005024157652848110452530069867406099383;

  long double Z = (long double)N;
  long double Sc = powl((Z + A), (Z + 0.5));
  Sc *= expl(-1.0 * (Z + A));
  Sc /= Z;

  long double F = 1.0;
  long double Ck;
  long double Sum = SQRT2PI;

  for(int K = 1; K < A; K++)
  {
    Z++;
    Ck = powl(A - K, K - 0.5);
    Ck *= expl(A - K);
    Ck /= F;

    Sum += (Ck / Z);

    F *= (-1.0 * K);
  }

  return (double)(Sum * Sc);
}

long double log_gamma(double N)
{
  /*
   * The constant SQRT2PI is defined as sqrt(2.0 * PI). For speed the constant is already defined in decimal form.
   * However, if you wish to ensure that you achieve maximum precision on your own machine, you can calculate it
   * yourself using (sqrt(atan(1.0) * 8.0))
   */

  // const long double SQRT2PI = sqrtl(atanl(1.0) * 8.0);
  const long double SQRT2PI = 2.5066282746310005024157652848110452530069867406099383;

  long double Z = (long double)N;
  long double Sc;

  Sc = (logl(Z + A) * (Z + 0.5)) - (Z + A) - logl(Z);

  long double F = 1.0;
  long double Ck;
  long double Sum = SQRT2PI;

  for(int K = 1; K < A; K++)
  {
    Z++;
    Ck = powl(A - K, K - 0.5);
    Ck *= expl(A - K);
    Ck /= F;

    Sum += (Ck / Z);

    F *= (-1.0 * K);
  }

  return logl(Sum) + Sc;
}

double approx_gamma(double Z)
{
  const double RECIP_E = 0.36787944117144232159552377016147;  // RECIP_E = (E^-1) = (1.0 / E)
  const double TWOPI = 6.283185307179586476925286766559;      // TWOPI = 2.0 * PI

  double D = 1.0 / (10.0 * Z);
  D = 1.0 / ((12 * Z) - D);
  D = (D + Z) * RECIP_E;
  D = pow(D, Z);
  D *= sqrt(TWOPI / Z);

  return D;
}

double approx_log_gamma(double N)
{
  const double LOGPIHALF = 0.24857493634706692717563414414545;  // LOGPIHALF = (log10(PI) / 2.0)

  double D;

  D = 1.0 + (2.0 * N);
  D *= 4.0 * N;
  D += 1.0;
  D *= N;
  D = log10(D) * (1.0 / 6.0);
  D += N + (LOGPIHALF);
  D = (N * log(N)) - D;
  return D;
}

/*
// Slightly faster
double approx_gamma(double Z)
{
const double RECIP_E = 0.36787944117144232159552377016147;  // RECIP_E =
(E^-1) = (1.0 / E) const double TWOPI = 6.283185307179586476925286766559;  //
TWOPI = 2.0 * PI const double RECIP_Z = (1.0 / Z);

double D = (0.1 * RECIP_Z);
D = 1.0 / ((12 * Z) - D);
D = (D + Z) * RECIP_E;
D = pow(D, Z);
D *= sqrt(TWOPI * RECIP_Z);

return D;
}
*/
