#include "DroneKeyboard.h"

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

#include "SimMode/SimModeBase.h"
#include "Vehicles/Multirotor/SimModeWorldMultiRotor.h"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"

#include <chrono>

using msr::airlib::MultirotorApiBase;

bool UDroneKeyboard::ShouldCreateSubsystem(UObject* Outer) const
{
	const UWorld* World = Cast<UWorld>(Outer);
	return World && World->IsGameWorld() && FParse::Param(FCommandLine::Get(), TEXT("DroneKeyboard"));
}

void UDroneKeyboard::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	FParse::Value(FCommandLine::Get(), TEXT("DroneKeyboardSpeed="), Speed);
	// Stop before AirSim's actors are torn down (they own the vehicle API).
	FWorldDelegates::OnWorldBeginTearDown.AddUObject(this, &UDroneKeyboard::Stop);
}

void UDroneKeyboard::Deinitialize()
{
	Stop();
	FWorldDelegates::OnWorldBeginTearDown.RemoveAll(this);
	Super::Deinitialize();
}

void UDroneKeyboard::Tick(float DeltaTime)
{
	// A status line at the top of the window: what it waits for, or the keys.
	auto Status = [this](const FString& Text, const FColor& Color) {
		if (GEngine)
			GEngine->AddOnScreenDebugMessage(reinterpret_cast<uint64>(this), 1.0f, Color, Text);
	};
	APlayerController* Player = GetWorld()->GetFirstPlayerController();
	if (!Player)
	{
		Status(TEXT("DroneKeyboard: no player controller"), FColor::Red);
		return;
	}
	auto Axis = [Player](const FKey& Plus, const FKey& Minus) {
		return (Player->IsInputKeyDown(Plus) ? 1.0f : 0.0f) - (Player->IsInputKeyDown(Minus) ? 1.0f : 0.0f);
	};
	Forward = Axis(EKeys::W, EKeys::S) * Speed;
	Right = Axis(EKeys::D, EKeys::A) * Speed;
	Down = Axis(EKeys::LeftShift, EKeys::SpaceBar) * Speed * 2 / 3;
	YawRate = Axis(EKeys::E, EKeys::Q) * 45.0f;
	if (Running)
		Status(FString::Printf(TEXT("DroneKeyboard: W A S D move, Space / Shift up / down, Q / E turn"
								   "   [forward %.0f  right %.0f  up %.0f  turn %.0f]"),
				   Forward.load(), Right.load(), -Down.load(), YawRate.load()),
			FColor::Green);

	// Start once AirSim's multirotor simulation is up.
	if (Running || Worker.joinable())
		return;
	ASimModeBase* Sim = ASimModeBase::getSimMode();
	msr::airlib::ApiProvider* Provider = Sim ? Sim->getApiProvider() : nullptr;
	if (!Provider || Provider->getVehicleApis().getMap().empty())
	{
		Status(FString::Printf(TEXT("DroneKeyboard: waiting for AirSim (%s)"),
				   !Sim ? TEXT("no simulation") : !Provider ? TEXT("no API") : TEXT("no vehicle")),
			FColor::Yellow);
		return;
	}
	// (Not AirSimSettings::singleton(): AirLib is header only, so this module
	// has its own, empty copy of it; the simulation's class tells the type.)
	if (!Cast<ASimModeWorldMultiRotor>(Sim))
	{
		Status(TEXT("DroneKeyboard: off, AirSim is not simulating a multirotor"), FColor::Yellow);
		return;
	}
	// The first vehicle (the only one in our settings).
	const auto& First = *Provider->getVehicleApis().getMap().begin();
	auto* Api = static_cast<MultirotorApiBase*>(First.second);
	Running = true;
	Worker = std::thread([this, Api] { Fly(Api); });
	UE_LOG(LogTemp, Warning, TEXT("DroneKeyboard: flying %hs"), First.first.c_str());
}

void UDroneKeyboard::Fly(MultirotorApiBase* Api)
{
	using namespace std::chrono_literals;
	try
	{
		Api->enableApiControl(true);
		Api->armDisarm(true);
	}
	catch (const std::exception&)
	{
	}
	bool Idle = false;
	while (Running)
	{
		const float F = Forward, R = Right, D = Down, Yaw = YawRate;
		const bool Still = F == 0 && R == 0 && D == 0 && Yaw == 0;
		if (Still && Idle)
		{
			// Hovering after the last stop command; leave AirSim to other
			// commands (ROS /cmd_vel, scripts) until a key is pressed.
			std::this_thread::sleep_for(20ms);
			continue;
		}
		Idle = Still;
		try
		{
			// Blocks for its duration (50 ms), then the keys are read again.
			Api->moveByVelocityBodyFrame(F, R, D, 0.05f, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
				msr::airlib::YawMode(true, Yaw));
		}
		catch (const std::exception&)
		{
			std::this_thread::sleep_for(50ms);  // e.g. API control taken away
		}
	}
}

void UDroneKeyboard::Stop(UWorld* World)
{
	if (World && World != GetWorld())
		return;
	Running = false;
	if (Worker.joinable())
		Worker.join();
}
