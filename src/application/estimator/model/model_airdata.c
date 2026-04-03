#include "application/estimator/model/model_airdata.h"
#include "common/math/math.h"
#include <math.h>

// physical constants
static const double air_R = 287.0579; // specific gas constant
static const double air_gamma = 1.4; // adiabatic index for air
static const double earth_r0 = 6356766.0; // mean earth radius
static const double earth_g0 = 9.81; // gravitational acceleration

typedef struct {
	double base_height; // altitude, for which following constants are defined for
	double base_pressure;
	double base_temperature;
	double base_temperature_lapse_rate; // temperature change with altitude
} atmosphere_layer_t; // struct for one layer of atmoshphere

static const atmosphere_layer_t air_atmosphere[] = {
	{0.0, 101325.00, 288.15, 0.0065}, // Troposphere
	{11000.0, 22632.10, 216.65, 0.0000}, // Tropopause
	{20000.0, 5474.90, 216.65, -0.0010}, // Stratosphere
	{32000.0, 868.02, 228.65, -0.0028} // Stratosphere 2
};

// altdata function uses barometric pressure to determine current altitude (inside Troposphere)
double model_altdata(double pressure) {
	double altitude = 0;

	// Select Troposphere
	const double b = air_atmosphere[0].base_height;
	const double P_B = air_atmosphere[0].base_pressure;
	const double T_B = air_atmosphere[0].base_temperature;
	const double k = air_atmosphere[0].base_temperature_lapse_rate;

	// inverse barometric formula, for Troposphere
	altitude = b + (T_B / k) * (1.0 - pow(pressure / P_B, (air_R * k) / earth_g0));

	// Geopotential altitude to normal altitude
	altitude = altitude * earth_r0 / (altitude + earth_r0);
	return altitude;
}

// from commit 7072518
// airdata function uses altitude to return pressure, temperature, density, local mach
estimator_airdata_t model_airdata(double altitude) {
	estimator_airdata_t result = {0};

	// Altitude to geopotential altitude
	altitude = earth_r0 * altitude / (earth_r0 - altitude);

	// Select atmosphere layer
	const atmosphere_layer_t *layer = &air_atmosphere[0]; // start with Troposphere
	if (altitude >
		air_atmosphere[1].base_height) // check if altitude is in higher atmosphere layers
	{
		if (altitude < air_atmosphere[2].base_height) {
			layer = &air_atmosphere[1];
		} else if (altitude < air_atmosphere[3].base_height) {
			layer = &air_atmosphere[2];
		} else {
			layer = &air_atmosphere[3];
		}
	}
	const double b = layer->base_height;
	const double P_B = layer->base_pressure;
	const double T_B = layer->base_temperature;
	const double k = layer->base_temperature_lapse_rate;

	// temperature
	result.temperature = T_B - k * (altitude - b);

	// static pressure, different barometric formulas for lapse rate / no lapse rate
	if (k == 0) {
		result.pressure = P_B * exp(-earth_g0 * (altitude - b) / (air_R * T_B));
	} else {
		result.pressure = P_B * pow(1.0f - (k / T_B) * (altitude - b), earth_g0 / (air_R * k));
	}
	if (float_equal(result.temperature, 0.0) || float_equal(P_B, 0.0) || float_equal(T_B, 0.0)) {
		return result;
	}
	// density
	result.density = result.pressure / (air_R * result.temperature);

	// local speed of sound
	if (air_gamma * air_R * result.temperature < 0.0 || result.temperature < 0.0) {
		return result;
	}
	result.mach_local = sqrt(air_gamma * air_R * result.temperature);

	return result;
}

double model_airdata_jacobian(double altitude) {
	// result
	double pressure_altitude;

	// geopotential altitude
	const double altitude_ratio = earth_r0 / (earth_r0 - altitude);
	altitude *= altitude_ratio;

	// Select atmosphere behavior from table
	const atmosphere_layer_t *layer = &air_atmosphere[0]; // start with Troposphere
	if (altitude >
		air_atmosphere[1].base_height) // check if altitude is in higher atmosphere layers
	{
		if (altitude < air_atmosphere[2].base_height) {
			layer = &air_atmosphere[1];
		} else if (altitude < air_atmosphere[3].base_height) {
			layer = &air_atmosphere[2];
		} else {
			layer = &air_atmosphere[3];
		}
	}
	const double b = layer->base_height;
	const double P_B = layer->base_pressure;
	const double T_B = layer->base_temperature;
	const double k = layer->base_temperature_lapse_rate;

	// static pressure, different barometric formulas for lapse rate / no lapse rate
	if (k == 0) {
		pressure_altitude = -P_B * earth_g0 / (T_B * air_R) * (altitude_ratio * altitude_ratio) *
							exp(-earth_g0 * (altitude - b) / (T_B * air_R));
	} else {
		pressure_altitude = -P_B * earth_g0 / (T_B * air_R) * (altitude_ratio * altitude_ratio) *
							pow(1 - k / T_B * (altitude - b), earth_g0 / (air_R * k) - 1);
	}

	return pressure_altitude;
}
