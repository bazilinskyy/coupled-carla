// Coupled CARLA: Google Photorealistic 3D Tiles in a CARLA level (see Public/CCPhotorealTiles.h).

using UnrealBuildTool;

public class CoupledPhotoreal : ModuleRules
{
  public CoupledPhotoreal(ReadOnlyTargetRules Target) : base(Target)
  {
    PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
    // the Carla module's headers (LibCarla, boost) want both, as CarlaTools sets them
    bEnableExceptions = true;
    bUseRTTI = true;
    CppStandard = CppStandardVersion.Cpp20;   // as CesiumRuntime

    PrivateDependencyModuleNames.AddRange(new string[]
    {
      "Core",
      "CoreUObject",
      "Engine",
      "Json",
      "HTTP",
      "RenderCore",   // GGameThreadTime, GRenderThreadTime (the profile rows)
      "RHI",          // RHIGetGPUFrameCycles
      "CesiumRuntime",
      "Carla",
      // what the Carla module's public headers include (CarlaEpisode.h -> vehicles, physics, navigation)
      "ChaosVehicles",
      "ChaosVehiclesCore",
      "PhysicsCore",
      "Chaos",
      "NavigationSystem",
      "PCG",
    });
  }
}
