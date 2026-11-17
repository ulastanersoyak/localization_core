// Flies the AirSim drone with the keyboard, in the Unreal window, as in a
// game: hold W A S D to move, Space / Shift to climb / descend, Q / E to turn;
// let go and it hovers. Only with -DroneKeyboard on Unreal's command line
// (airsim/run.sh drone passes it).
//
// The keys are read every frame; a worker thread sends them to AirSim's own
// vehicle API as velocity commands (moveByVelocityBodyFrame, the call behind
// the client API of the same name), because that call blocks for its
// duration and must not stall the game thread. Speeds: -DroneKeyboardSpeed=
// (m/s, default 3; climbing at two thirds of it, turning 45 deg/s).

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"

#include <atomic>
#include <thread>

#include "DroneKeyboard.generated.h"

namespace msr { namespace airlib { class MultirotorApiBase; } }

UCLASS()
class UDroneKeyboard : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	bool ShouldCreateSubsystem(UObject* Outer) const override;
	void Initialize(FSubsystemCollectionBase& Collection) override;
	void Deinitialize() override;
	void Tick(float DeltaTime) override;
	TStatId GetStatId() const override { RETURN_QUICK_DECLARE_CYCLE_STAT(UDroneKeyboard, STATGROUP_Tickables); }

private:
	void Stop(UWorld* World = nullptr);
	void Fly(msr::airlib::MultirotorApiBase* Api);

	// Commanded body velocity (AirSim: forward, right, down; m/s) and yaw
	// rate (deg/s), written by the game thread, read by the worker.
	std::atomic<float> Forward{ 0 }, Right{ 0 }, Down{ 0 }, YawRate{ 0 };
	std::atomic<bool> Running{ false };
	std::thread Worker;
	float Speed = 3.0f;
};
