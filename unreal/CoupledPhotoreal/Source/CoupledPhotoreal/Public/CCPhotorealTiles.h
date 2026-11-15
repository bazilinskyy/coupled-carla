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
//   4. spawns a CesiumGeoreference, one CesiumCartographicPolygon per carve ring and the Cesium3DTileset (no physics
//      meshes: collision stays CARLA's), with a polygon clipping overlay that cuts the photogrammetry under the roads;
//   5. hides the twin's building and ground meshes (visibility only: collision, LiDAR and navigation keep them).
// Anything missing (key, pack data, caps, a load error) leaves the level as it was, logs one plain line and destroys
// the actor, which the client sees. Destroying it (or the next load_world) restores the twin.
//
// Options arrive in the actor's role_name as "key=value;key=value" (coupledcarla/photoreal.py role_options()).

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UObject/ObjectKey.h"
#include "CCPhotorealTiles.generated.h"

class ACesium3DTileset;
class ACesiumGeoreference;
class ACesiumCartographicPolygon;
class UCesiumPolygonRasterOverlay;
struct FCesium3DTilesetLoadFailureDetails;

DECLARE_LOG_CATEGORY_EXTERN(LogCoupledPhotoreal, Log, All);

UCLASS(NotBlueprintable, NotPlaceable)
class COUPLEDPHOTOREAL_API ACCPhotorealTiles_C : public AActor
{
  GENERATED_BODY()

public:
  ACCPhotorealTiles_C();

  virtual void Tick(float DeltaSeconds) override;

  /// The key as found on this machine ("" when there is none) and where it came from ("env", "file", "").
  static FString ReadKey(FString &Source);

  /// ~/.config/coupled-carla on this machine.
  static FString ConfigDir();

protected:
  virtual void BeginPlay() override;
  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
  enum class EState : uint8 { Waiting, Running, Stopped };

  bool Start(FString &Why);
  bool LoadLevelData(const FString &MapName, FString &Why);
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
  bool bTestRefreshed = false;
  double NextCredits = 0.0;
};
