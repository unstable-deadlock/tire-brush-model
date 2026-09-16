#include "TireBrushModel.h"
#include "Math/UnrealMathUtility.h"

ATireBrushModel::ATireBrushModel()
{
	PrimaryActorTick.TickInterval = 0.001f; // 1000 Hz = 1ms
	PrimaryActorTick.TickType = ETickingGroup::TG_PrePhysics;
}

void ATireBrushModel::BeginPlay()
{
	Super::BeginPlay();
	CurrentTireState = FTireState();
	CurrentForces = FTireForces();
}

void ATireBrushModel::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	UpdateTireForces(CurrentTireState, CurrentForces);
	UpdateCount++;
}

void ATireBrushModel::UpdateTireForces(const FTireState& InTireState, FTireForces& OutForces)
{
	CurrentTireState = InTireState;

	// Temperature-dependent friction coefficient
	float Friction = CalculateFrictionCoefficient(InTireState.TireTemperature);

	// Longitudinal force using brush model
	OutForces.LongitudinalForce = CalculateLongitudinalForce(
		InTireState.LongitudinalSlip,
		InTireState.VerticalLoad,
		Friction
	);

	// Lateral force using brush model
	OutForces.LateralForce = CalculateLateralForce(
		InTireState.LateralSlip,
		InTireState.VerticalLoad,
		Friction
	);

	// Self-aligning torque (simplified)
	OutForces.AlignmentTorque = CalculateAlignmentTorque(OutForces.LateralForce, InTireState.VerticalLoad);

	// Rolling resistance
	OutForces.RollingResistance = InTireState.VerticalLoad * 0.015f;

	CurrentForces = OutForces;
	LastUpdateTime = FPlatformTime::Seconds();
}

void ATireBrushModel::FastUpdateTireForces(
	float LongitudinalSlip,
	float LateralSlip,
	float VerticalLoad,
	float TireTemperature,
	float DeltaTime,
	float& OutLongitudinalForce,
	float& OutLateralForce,
	float& OutAlignmentTorque)
{
	// Ultra-fast version for Blueprint use (< 0.5ms per call)
	float Friction = CalculateFrictionCoefficient(TireTemperature);

	OutLongitudinalForce = CalculateLongitudinalForce(LongitudinalSlip, VerticalLoad, Friction);
	OutLateralForce = CalculateLateralForce(LateralSlip, VerticalLoad, Friction);
	OutAlignmentTorque = CalculateAlignmentTorque(OutLateralForce, VerticalLoad);
}

FORCEINLINE float ATireBrushModel::CalculateFrictionCoefficient(float Temperature) const
{
	// Parabolic temperature sensitivity
	// Peak friction at optimal temperature
	float TempDelta = Temperature - TireConfiguration.OptimalTemperature;
	float FrictionReduction = TireConfiguration.TemperatureSensitivity * (TempDelta * TempDelta) * 0.0001f;
	
	return FMath::Max(TireConfiguration.SlidingFrictionCoefficient,
		TireConfiguration.PeakFrictionCoefficient - FrictionReduction);
}

FORCEINLINE float ATireBrushModel::CalculateLongitudinalForce(
	float Slip,
	float Load,
	float Friction) const
{
	// Simplified Pacejka brush model for longitudinal force
	// F = mu * Fz * (2/pi) * arctan(Cs * kappa)
	// Optimized for speed: avoid expensive trig where possible

	if (FMath::Abs(Slip) < 0.001f)
	{
		return 0.0f;
	}

	// Linear region (low slip)
	if (FMath::Abs(Slip) < 0.1f)
	{
		return TireConfiguration.LongitudinalStiffness * Slip;
	}

	// Nonlinear region (moderate to high slip)
	float SlipClamped = FMath::Clamp(Slip, -1.0f, 1.0f);
	float Force = Friction * Load * (2.0f / PI) * FMath::Atan(10.0f * SlipClamped);

	return Force;
}

FORCEINLINE float ATireBrushModel::CalculateLateralForce(
	float Slip,
	float Load,
	float Friction) const
{
	// Simplified Pacejka brush model for lateral force
	// Similar structure to longitudinal but with lateral stiffness

	if (FMath::Abs(Slip) < 0.001f)
	{
		return 0.0f;
	}

	// Linear region (small slip angles)
	if (FMath::Abs(Slip) < 0.05f)
	{
		return TireConfiguration.LateralStiffness * Slip;
	}

	// Nonlinear region
	float SlipClamped = FMath::Clamp(Slip, -0.5f, 0.5f);
	float Force = Friction * Load * (2.0f / PI) * FMath::Atan(12.0f * SlipClamped);

	return Force;
}

FORCEINLINE float ATireBrushModel::CalculateAlignmentTorque(
	float LateralForce,
	float Load) const
{
	// Self-aligning torque (simplified linear model)
	// Mz = Fy * Cp * (Fz/Fz0)
	// Cp = pneumatic trail

	if (Load < 100.0f)
	{
		return 0.0f;
	}

	float PneumaticTrail = TireConfiguration.TireRadius * 0.3f; // ~30% of tire radius
	float Torque = LateralForce * PneumaticTrail * (Load / 5000.0f);

	return FMath::Clamp(Torque, -5000.0f, 5000.0f);
}
