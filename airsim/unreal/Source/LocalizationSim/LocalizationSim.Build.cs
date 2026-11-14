using System.IO;
using UnrealBuildTool;

// The project's module. Cosys-AirSim's documentation requires a C++ module
// that depends on the AirSim plugin (docs/unreal_proj.md); it also holds
// DroneKeyboard (flying the drone with the keyboard in the Unreal window).
public class LocalizationSim : ModuleRules
{
	public LocalizationSim(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		bEnableExceptions = true;  // AirSim's API reports errors as exceptions
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "AirSim" });
		// AirSim's headers (SimMode/...) live in its module folder, not Public/.
		PrivateIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "..", "Plugins", "AirSim", "Source"));
	}
}
