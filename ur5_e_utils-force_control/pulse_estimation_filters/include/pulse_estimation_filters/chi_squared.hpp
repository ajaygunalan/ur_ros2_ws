#ifndef PULSE_CHI_SQUARED_HPP
#define PULSE_CHI_SQUARED_HPP

/**
 *
 * \brief Incomplete Gamma Function
 *
 * \details No longer need as I'm now using the log_igf(), but I'll leave this here anyway.
 *
 */
static double igf(double S, double Z);

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
static long double log_igf(long double S, long double Z);
static long double KM(long double S, long double Z);

double chisqr(int Dof, double Cv);

/**
 *
 * \brief Implementation of the Gamma function using Spouge's Approximation in C.
 *
 */
double gamma(double Z);
long double log_gamma(double N);

double approx_gamma(double z);
double approx_log_gamma(double Z);

#endif /* PULSE_CHI_SQUARED_HPP */
