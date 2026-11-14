using UnrealBuildTool;

public class LocalizationSimEditorTarget : TargetRules
{
	public LocalizationSimEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.V2;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_2;
		ExtraModuleNames.Add("LocalizationSim");
	}
}
