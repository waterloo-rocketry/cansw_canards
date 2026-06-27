/**
 * Atmospheric model
 * uses US standard atmosphere, after: Stengel 2004 - Flight Dynamics
 */
#ifndef MODEL_AIRDATA_H
#define MODEL_AIRDATA_H

typedef struct {
	double pressure;
	double temperature;
	double density;
	double mach_local;
} estimator_airdata_t;

// altdata function uses barometric pressure to determine current altitude (inside Troposphere)
double model_altdata(double pressure);

// airdata function uses altitude to return pressure, temperature, density, local mach
estimator_airdata_t model_airdata(double altitude);

/**
 * Jacobian of the airdata function (partial derivative of sub-function)
 * @param double altitude
 * @return double pressure change with respect to altitude
 */
double model_airdata_jacobian(double altitude);

//   void model_airdata_biasfinder(void)

#endif
