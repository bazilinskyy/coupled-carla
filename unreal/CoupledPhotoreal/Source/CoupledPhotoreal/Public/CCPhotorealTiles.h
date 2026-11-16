// Coupled CARLA: Google Photorealistic 3D Tiles under CARLA's own roads (map.photoreal).
//
// A CARLA client spawns this actor as `blueprint.cc_photoreal_google` (listed in the CoupledCarla content pack's
// Content/Config/BlueprintParameters.json, server started with -carla-packs=<pack>). CARLA's BlueprintActorFactory
// appends "_C" to a catalogue path unless it already ends in it, so the native class is named with that suffix and the
// catalogue gives "/Script/CoupledPhotoreal.CCPhotorealTiles_C": no Blueprint asset is needed.
//
// On its first ticks it:
//   1. reads the Google Maps Platform key on THIS (the server's) machine: $GOOGLE_MAPS_API_KEY, else
//      ~/.config/coupled-carla/google_maps_key (Windows: %USERPROFILE%\.config\coupled-carla\google_maps_key). The key is
//      sent as the X-Goog-Api-Key header, never put in a URL, a log line or an asset;
//   2. checks the usage ledger (~/.config/coupled-carla/google_tiles_usage.json, shared with coupledcarla/photoreal.py):
//      one root tileset request (the billed unit, a session of up to three hours) per spawn, with daily and monthly caps;
//   3. reads the level's data written by `python -m coupledcarla.photoreal prepare` into a content pack
//      (Content/Config/Photoreal/<Level>.json): the georeference origin (WGS84 longitude, latitude, ellipsoidal height of
//      the level's 0,0,0), the road carve polygons and the building footprints;
//   4. spawns a CesiumGeoreference and the Cesium3DTileset (no physics meshes: collision stays CARLA's) and cuts the
//      photogrammetry away over the twin's lanes, by one of
//      - "mask" (default): the tile material M_CCPhotorealTiles (this plugin's content) samples a precomputed grid
//        (<Level>_cut.bin: signed distance to the cut's outline, road height) and clips only up to cut_top metres
//        above the road, so tree crowns and building overhangs stay. One texture for the level, no per-tile work;
//      - "polygons": Cesium's polygon clipping overlay with one CesiumCartographicPolygon per carve ring, rasterised
//        per tile on Cesium's worker threads (full height; the old way, kept for comparison and as the fallback);
//   5. hides the twin's building and ground meshes (visibility only: collision, LiDAR and navigation keep them);
//   6. hands CARLA's camera sensors (their scene captures) to Cesium's camera manager, so tiles load where the
//      stations', drones' and film cameras look rather than only for the spectator (option cams), and optionally stops
//      the server's own window from rendering the world (viewport=0; the sensors render on their own).
// Profiling and A/B tests (one level load, so one billed root request): prof=<s> appends frame timings and tile counts
// to google_tiles_profile.csv beside the usage ledger; control=1 applies photoreal_control.txt there at run time
// (tools/photoreal_profile.py writes it).
// Anything missing (key, pack data, caps, a load error) leaves the level as it was, logs one plain line and destroys
// the actor, which the client sees. Destroying it (or the next load_world) restores the twin.
//
// Options arrive in the actor's role_name as "key=value;key=value" (coupledcarla/photoreal.py role_options()).

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UObject/ObjectKey.h"
#include "Cesium3DTilesetLifecycleEventReceiver.h"
#include "CCPhotorealTiles.generated.h"

class ACesium3DTileset;
class ACesiumGeoreference;
class ACesiumCartographicPolygon;
class UCesiumPolygonRasterOverlay;
class UMaterialInstanceDynamic;
class UMaterialInterface;
class UTexture2D;
class USceneCaptureComponent2D;
struct FCesium3DTilesetLoadFailureDetails;

DECLARE_LOG_CATEGORY_EXTERN(LogCoupledPhotoreal, Log, All);

UCLASS(NotBlueprintable, NotPlaceable)
class COUPLEDPHOTOREAL_API ACCPhotorealTiles_C : public AActor, public ICesium3DTilesetLifecycleEventReceiver
{
  GENERATED_BODY()

public:
  ACCPhotorealTiles_C();

  virtual void Tick(float DeltaSeconds) override;

  /// The key as found on this machine ("" when there is none) and where it came from ("env", "file", "").
  static FString ReadKey(FString &Source);

  /// ~/.config/coupled-carla on this machine.
  static FString ConfigDir();

  // ICesium3DTilesetLifecycleEventReceiver: the cut grid on each tile's material, tile counts for the profile
  virtual void CustomizeMaterial(ICesiumLoadedTilePrimitive &TilePrimitive, UMaterialInstanceDynamic &Material,
                                 const UCesiumMaterialUserData *CesiumData,
                                 const CesiumGltf::Material &GltfMaterial) override;
  virtual void OnTileVisibilityChanged(ICesiumLoadedTile &Tile, bool bVisible) override;
  virtual void OnTileLoaded(ICesiumLoadedTile &Tile) override;
  virtual void OnTileUnloading(ICesiumLoadedTile &Tile) override;

protected:
  virtual void BeginPlay() override;
  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
  enum class EState : uint8 { Waiting, Running, Stopped };

  bool Start(FString &Why);
  bool LoadLevelData(const FString &MapName, FString &Why);
  bool LoadCutGrid(const TSharedPtr<FJsonObject> &Cut, const FString &Folder, FString &Why);
  void ApplyCutParams(UMaterialInstanceDynamic *Mid) const;
  void ApplyCutParamsToAll();
  void SetCutMode(const FString &Mode);
  void UpdateCameras(double T);
  void SetViewport(bool bOn);
  void ApplyNativeOptions();
  void UpdateProfile(double T, float DeltaSeconds);
  void FlushProfile(double T);
  void CheckControl(double T);
  void ApplyRuntime(const FString &Key, const FString &Value);
  void SpawnCesium(const FString &Key);
  void SetClip(bool bWithFootprints);
  void ScanTwinMeshes();
  void ApplyVisibility();
  void UpdateNight();
  void CheckReload();
  void UpdateCredits();
  void StopTiles(const FString &Why);
  void RestoreTwin();
  void OnLoadFailure(const FCesium3DTilesetLoadFailureDetails &Details);
  double Now() const;
  FString Opt(const TCHAR *Name, const TCHAR *Default) const;
  double OptNum(const TCHAR *Name, double Default) const;

  // usage ledger (the same JSON file coupledcarla/photoreal.py Usage reads and writes)
  bool LedgerAllows(FString &Why) const;
  void LedgerCountRoot();

  EState State = EState::Waiting;
  double SpawnTime = 0.0;
  double StartTime = 0.0;
  double NextScan = 0.0;
  FString MapName;
  FString RunStart;
  TMap<FString, FString> Options;

  double OriginLon = 0.0, OriginLat = 0.0, OriginHeight = 0.0;
  TArray<TArray<FVector>> CarveRings;        // longitude, latitude, 0
  TArray<TArray<FVector>> FootprintRings;    // longitude, latitude, 0
  TArray<FVector2D> Coverage;                // level XY (cm); empty: the whole level
  TArray<FString> HideFolders;               // mesh paths containing one of these (and /Twins/) are hidden

  UPROPERTY(Transient)
  TObjectPtr<ACesiumGeoreference> Georeference;

  UPROPERTY(Transient)
  TObjectPtr<ACesium3DTileset> Tileset;

  UPROPERTY(Transient)
  TArray<TObjectPtr<ACesiumCartographicPolygon>> CarvePolygons;

  UPROPERTY(Transient)
  TArray<TObjectPtr<ACesiumCartographicPolygon>> FootprintPolygons;

  UPROPERTY(Transient)
  TObjectPtr<UCesiumPolygonRasterOverlay> Clip;

  struct FManaged
  {
    TWeakObjectPtr<AActor> Actor;
    bool bBuilding = false;
  };
  TArray<FManaged> Managed;
  TSet<FObjectKey> Seen;

  const void *NativeTileset = nullptr;
  int32 RootsThisRun = 0;
  int32 RunCap = 2;
  bool bNight = false;
  bool bNightKnown = false;
  bool bNightBuildings = true;
  bool bClipFootprints = false;
  FDelegateHandle FailureHandle;
  FString LastCredits;
  FString PendingStop;
  int32 LastHidden = -1;
  // hold (option "hold", % of Cesium's load progress): the twin stays visible until the tiles are this far loaded, so
  // nothing shows blank holes before the first tiles arrive; "hold_low" (% , 0 = off) shows it again below that
  bool bTilesReady = false;
  double ReadySince = -1.0;
  void UpdateHold(double T);
  bool bTestRefreshed = false;
  double NextCredits = 0.0;

  // the cut: "mask" (M_CCPhotorealTiles + the grid), "polygons" (Cesium's overlay), "none"
  FString CutMode;
  bool bMaskReady = false;
  double CutTop = 2.8, CutGrow = 0.0;
  FLinearColor CutXform = FLinearColor(0, 0, 0, 0);
  UPROPERTY(Transient)
  TObjectPtr<UTexture2D> CutTexture;
  UPROPERTY(Transient)
  TObjectPtr<UMaterialInterface> TileMaterial;
  TArray<TWeakObjectPtr<UMaterialInstanceDynamic>> Mids;

  // tiles seen by the lifecycle receiver
  TSet<const void *> VisibleTiles;
  int32 LoadedTiles = 0;
  bool bTilesShown = true;

  // CARLA's camera sensors fed to Cesium's camera manager
  bool bFeedCameras = true;
  FString PlayerCameras = TEXT("auto");
  double NextCamScan = 0.0;
  TArray<TWeakObjectPtr<USceneCaptureComponent2D>> Captures;
  int32 LastCams = -1;
  bool bViewportOff = false;
  bool bNativeOptionsApplied = false;

  // profile rows (google_tiles_profile.csv) and the control file (photoreal_control.txt)
  double ProfEvery = 0.0, NextProf = 0.0, ProfStart = 0.0, ProfLastTick = 0.0;
  int64 ProfFrames = 0;
  double ProfWall = 0.0, ProfGame = 0.0, ProfRender = 0.0, ProfRhi = 0.0, ProfGpu = 0.0, ProfMaxFrame = 0.0;
  FString Phase = TEXT("start");
  bool bControl = false;
  double NextControl = 0.0;
  FString LastControl;
};
