#ifndef FLUID_PRESSURE_H_
#define FLUID_PRESSURE_H_

namespace FluidPressure
{
	const double PascalsPerAtmosphere = 101325.0;
	const double MetersPerAtmosphere = 10.0;
	const double MetersPerPascal = MetersPerAtmosphere / PascalsPerAtmosphere;
	const double PressureAtSurface = PascalsPerAtmosphere;

	/// Get water depth in meters from pressure in Pascals.
	///
	double getDepth(double pressure)
	{
		return (pressure - PressureAtSurface) * MetersPerPascal;
	}

	/// Get water depth in meters from pressure in Pascals using provided surface pressure.
	///
	double getDepth(double pressure, double pressureAtSurface)
	{
		return (pressure - pressureAtSurface) * MetersPerPascal;
	}

	/// Get depth in meters from Pascals unadjusted for surface pressure.
	///
	double getUnadjustedDepth(double pressure)
	{
		return pressure * MetersPerPascal;
	}
}

#endif // FLUID_PRESSURE_H_
