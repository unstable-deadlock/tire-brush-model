#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TireBrushModel.generated.h"

/**
 * Fast tire brush model for sim racing at 1000 Hz
 * Optimized for real-time performance in UE5.8
 */

USTRUCT(BlueprintType)
struct FTireState
{
	GENERATED_BODY()

	// Tire slip angles and ratios
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire State")
	float LongitudinalSlip = 0.0f; // kappa: -1 to +1

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire State")
	float LateralSlip = 0.0f; // alpha: slip angle in radians

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire State")
	float VerticalLoad = 5000.0f; // Load in Newtons

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire State")
	float CamberAngle = 0.0f; // Camber in radians

	// Tire velocities
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire State")
	FVector TireVelocity = FVector::ZeroVector;

	// Temperature (affects friction)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire State")
	float TireTemperature = 80.0f; // Celsius
};

USTRUCT(BlueprintType)
struct FTireForces
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Tire Forces")
	float LongitudinalForce = 0.0f; // Fx in Newtons

	UPROPERTY(BlueprintReadOnly, Category = "Tire Forces")
	float LateralForce = 0.0f; // Fy in Newtons

	UPROPERTY(BlueprintReadOnly, Category = "Tire Forces")
	float AlignmentTorque = 0.0f; // Mz in Newton-meters

	UPROPERTY(BlueprintReadOnly, Category = "Tire Forces")
	float RollingResistance = 0.0f; // Resistance torque
};

USTRUCT(BlueprintType)
struct FContactPatch
{
	GENERATED_BODY()

	// Contact patch dimensions
	UPROPERTY(BlueprintReadOnly, Category = "Contact Patch")
	float Length = 0.0f; // Length in longitudinal direction (m)

	UPROPERTY(BlueprintReadOnly, Category = "Contact Patch")
	float Width = 0.0f; // Width in lateral direction (m)

	UPROPERTY(BlueprintReadOnly, Category = "Contact Patch")
	float Area = 0.0f; // Contact area (m²)

	UPROPERTY(BlueprintReadOnly, Category = "Contact Patch")
	float PressureDistribution = 0.0f; // Average pressure in contact patch (Pa)

	// Slip characteristics in contact patch
	UPROPERTY(BlueprintReadOnly, Category = "Contact Patch")
	float SlidingRegionRatio = 0.0f; // Ratio of sliding area to total area (0-1)

	UPROPERTY(BlueprintReadOnly, Category = "Contact Patch")
	float MaxContactPressure = 0.0f; // Peak pressure (Pa)

	// Deformation
	UPROPERTY(BlueprintReadOnly, Category = "Contact Patch")
	float MaxDeformation = 0.0f; // Maximum tread deformation (m)
};

USTRUCT(BlueprintType)
struct FTireConfig
{
	GENERATED_BODY()

	// Tire dimensions
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Config")
	float TireRadius = 0.35f; // meters

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Config")
	float TireWidth = 0.25f; // meters

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Config")
	float AspectRatio = 0.65f; // Height/Width ratio

	// Contact patch parameters
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Config")
	float ContactPatchLengthCoefficient = 1.6f; // Calibration for length calculation

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Config")
	float TireStiffness = 250000.0f; // Vertical stiffness (N/m)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Config")
	float ReferenceLoad = 5000.0f; // Reference vertical load (N)

	// Friction coefficients (temperature dependent)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Config")
	float PeakFrictionCoefficient = 1.3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Config")
	float SlidingFrictionCoefficient = 0.9f;

	// Brush model stiffness
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Config")
	float LongitudinalStiffness = 100000.0f; // N

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Config")
	float LateralStiffness = 80000.0f; // N

	// Thermal model
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Config")
	float OptimalTemperature = 80.0f; // Celsius

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Config")
	float TemperatureSensitivity = 0.005f; // friction change per degree C
};

UCLASS()
class TIREBRUSH_API ATireBrushModel : public AActor
{
	GENERATED_BODY()

public:
	ATireBrushModel();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

	// Blueprint exposed calculation function
	UFUNCTION(BlueprintCallable, Category = "Tire Physics")
	void UpdateTireForces(const FTireState& InTireState, FTireForces& OutForces);

	// Calculate forces with minimal overhead (< 1ms at 1000 Hz)
	UFUNCTION(BlueprintCallable, Category = "Tire Physics")
	void FastUpdateTireForces(
		float LongitudinalSlip,
		float LateralSlip,
		float VerticalLoad,
		float TireTemperature,
		float DeltaTime,
		float& OutLongitudinalForce,
		float& OutLateralForce,
		float& OutAlignmentTorque
	);

	// Calculate contact patch properties
	UFUNCTION(BlueprintCallable, Category = "Tire Physics")
	void CalculateContactPatch(
		float VerticalLoad,
		float LongitudinalSlip,
		float LateralSlip,
		FContactPatch& OutContactPatch
	);

	// Configuration
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Config")
	FTireConfig TireConfiguration;

	// Diagnostic/debug
	UPROPERTY(BlueprintReadOnly, Category = "Debug")
	float LastUpdateTime = 0.0f;

	UPROPERTY(BlueprintReadOnly, Category = "Debug")
	int32 UpdateCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Debug")
	FContactPatch CurrentContactPatch;

protected:
	// Internal fast calculations
	FORCEINLINE float CalculateFrictionCoefficient(float Temperature) const;
	FORCEINLINE float CalculateLongitudinalForce(float Slip, float Load, float Friction) const;
	FORCEINLINE float CalculateLateralForce(float Slip, float Load, float Friction) const;
	FORCEINLINE float CalculateAlignmentTorque(float LateralForce, float Load) const;

	// Contact patch calculations
	FORCEINLINE float CalculateContactPatchLength(float VerticalLoad) const;
	FORCEINLINE float CalculateContactPatchWidth(float VerticalLoad) const;
	FORCEINLINE float CalculateContactPressure(float VerticalLoad, float PatchArea) const;
	FORCEINLINE float CalculateSlidingRegionRatio(float LongitudinalSlip, float LateralSlip) const;
	FORCEINLINE float CalculateMaxDeformation(float VerticalLoad) const;

	// State tracking
	FTireState CurrentTireState;
	FTireForces CurrentForces;
};
