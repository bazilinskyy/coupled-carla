// Coupled CARLA: Google Photorealistic 3D Tiles under CARLA's own roads. See CCPhotorealTiles.h.

#include "CCPhotorealTiles.h"

#include "Cesium3DTileset.h"
#include "CesiumCamera.h"
#include "CesiumCameraManager.h"
#include "CesiumLoadedTile.h"
#include "Cesium3DTilesetLoadFailureDetails.h"
#include "CesiumCartographicPolygon.h"
#include "CesiumCreditSystem.h"
#include "CesiumGeoreference.h"
#include "CesiumGlobeAnchorComponent.h"
#include "CesiumPolygonRasterOverlay.h"
#include "Cesium3DTilesSelection/Tileset.h"
#include "CesiumUtility/CreditSystem.h"

#include "Carla/Actor/CarlaActor.h"
#include "Carla/ContentPacks/ContentPackManager.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Weather/Weather.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "DynamicRHI.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/SceneCapture2D.h"
#include "Engine/Texture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "HAL/IConsoleManager.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Misc/Compression.h"
#include "RenderTimer.h"
#include "UObject/UObjectIterator.h"
#include "Dom/JsonObject.h"
#include "Engine/StaticMesh.h"
#include "GenericPlatform/GenericPlatformHttp.h"
#include "Engine/StaticMeshActor.h"
#include "EngineUtils.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformTime.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

IMPLEMENT_MODULE(FDefaultModuleImpl, CoupledPhotoreal);
DEFINE_LOG_CATEGORY(LogCoupledPhotoreal);

namespace
{
const TCHAR *GoogleRootUrl = TEXT("https://tile.googleapis.com/v1/3dtiles/root.json");
// the plugin's tile material (Content/Materials; tools in unreal/CoupledPhotoreal/Tools/make_material.py)
const TCHAR *TileMaterialPath = TEXT("/CoupledPhotoreal/Materials/M_CCPhotorealTiles.M_CCPhotorealTiles");
constexpr int32 MaxCameras = 16;
constexpr int32 MaxRunsKept = 200;

FString HomeDir()
{
#if PLATFORM_WINDOWS
  return FPlatformMisc::GetEnvironmentVariable(TEXT("USERPROFILE"));
#else
  return FPlatformMisc::GetEnvironmentVariable(TEXT("HOME"));
#endif
}

bool ReadJsonFile(const FString &Path, TSharedPtr<FJsonObject> &Out)
{
  FString Text;
  if (!FFileHelper::LoadFileToString(Text, *Path))
  {
    return false;
  }
  TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Text);
  return FJsonSerializer::Deserialize(Reader, Out) && Out.IsValid();
}

bool WriteJsonFile(const FString &Path, const TSharedRef<FJsonObject> &Object)
{
  FString Text;
  TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Text);
  if (!FJsonSerializer::Serialize(Object, Writer))
  {
    return false;
  }
  IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
  const FString Tmp = Path + TEXT(".tmp");
  if (!FFileHelper::SaveStringToFile(Text, *Tmp))
  {
    return false;
  }
  return IFileManager::Get().Move(*Path, *Tmp, true, true);
}

TArray<TArray<FVector>> RingsFromJson(const TSharedPtr<FJsonObject> &Root, const TCHAR *Field)
{
  TArray<TArray<FVector>> Rings;
  const TArray<TSharedPtr<FJsonValue>> *JRings = nullptr;
  if (!Root->TryGetArrayField(Field, JRings))
  {
    return Rings;
  }
  for (const TSharedPtr<FJsonValue> &JRing : *JRings)
  {
    TArray<FVector> Ring;
    for (const TSharedPtr<FJsonValue> &JPoint : JRing->AsArray())
    {
      const TArray<TSharedPtr<FJsonValue>> &P = JPoint->AsArray();
      if (P.Num() >= 2)
      {
        Ring.Add(FVector(P[0]->AsNumber(), P[1]->AsNumber(), 0.0));
      }
    }
    if (Ring.Num() >= 3)
    {
      Rings.Add(MoveTemp(Ring));
    }
  }
  return Rings;
}

bool InPolygon(const TArray<FVector2D> &Poly, const FVector2D &P)
{
  bool bIn = false;
  for (int32 i = 0, j = Poly.Num() - 1; i < Poly.Num(); j = i++)
  {
    const FVector2D &A = Poly[i];
    const FVector2D &B = Poly[j];
    if (((A.Y > P.Y) != (B.Y > P.Y)) && (P.X < (B.X - A.X) * (P.Y - A.Y) / (B.Y - A.Y) + A.X))
    {
      bIn = !bIn;
    }
  }
  return bIn;
}

FString SemanticFolder(const FString &Name)
{
  // /Game/Carla/Static/<Semantic>/Twins/<Level>/: the folder ATagger reads (carla-digitaltwins bake_level.py)
  static const TMap<FString, FString> Folders = {
      {TEXT("building"), TEXT("/Static/Building/")},
      {TEXT("terrain"), TEXT("/Static/Terrain/")},
      {TEXT("ground"), TEXT("/Static/Ground/")},
      {TEXT("sidewalk"), TEXT("/Static/SideWalk/")},
      {TEXT("road"), TEXT("/Static/Road/")},
      {TEXT("roadline"), TEXT("/Static/RoadLine/")},
      {TEXT("vegetation"), TEXT("/Static/Vegetation/")},
  };
  const FString *F = Folders.Find(Name.ToLower());
  return F ? *F : FString::Printf(TEXT("/Static/%s/"), *Name);
}

FString Str(const TSharedPtr<FJsonObject> &O, const TCHAR *Name)
{
  FString V;
  return O.IsValid() && O->TryGetStringField(Name, V) ? V : FString();
}

double Num(const TSharedPtr<FJsonObject> &O, const TCHAR *Name)
{
  double V = 0.0;
  return O.IsValid() && O->TryGetNumberField(Name, V) ? V : 0.0;
}

FString StripHtml(const FString &Html)
{
  FString Out;
  bool bTag = false;
  for (TCHAR C : Html)
  {
    if (C == '<')
      bTag = true;
    else if (C == '>')
      bTag = false;
    else if (!bTag)
      Out.AppendChar(C);
  }
  return Out.TrimStartAndEnd();
}
} // namespace

static FString LedgerPath(bool bTest);

ACCPhotorealTiles_C::ACCPhotorealTiles_C()
{
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.bStartWithTickEnabled = true;
  PrimaryActorTick.TickInterval = 0.0f;
  RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
  SetActorEnableCollision(false);
}

FString ACCPhotorealTiles_C::ConfigDir()
{
  // COUPLED_CARLA_CONFIG_DIR: another folder for the key, ledger and credits (tests; shared lab machines)
  const FString Override = FPlatformMisc::GetEnvironmentVariable(TEXT("COUPLED_CARLA_CONFIG_DIR"));
  if (!Override.IsEmpty())
  {
    return Override;
  }
  return FPaths::Combine(HomeDir(), TEXT(".config"), TEXT("coupled-carla"));
}

FString ACCPhotorealTiles_C::ReadKey(FString &Source)
{
  Source.Empty();
  FString Key = FPlatformMisc::GetEnvironmentVariable(TEXT("GOOGLE_MAPS_API_KEY")).TrimStartAndEnd();
  if (!Key.IsEmpty())
  {
    Source = TEXT("env");
    return Key;
  }
  FString Text;
  if (FFileHelper::LoadFileToString(Text, *FPaths::Combine(ConfigDir(), TEXT("google_maps_key"))))
  {
    TArray<FString> Lines;
    Text.ParseIntoArrayLines(Lines, true);
    for (const FString &Line : Lines)
    {
      Key = Line.TrimStartAndEnd();
      if (!Key.IsEmpty() && !Key.StartsWith(TEXT("#")))
      {
        Source = TEXT("file");
        return Key;
      }
    }
  }
  return FString();
}

double ACCPhotorealTiles_C::Now() const
{
  return FPlatformTime::Seconds();
}

FString ACCPhotorealTiles_C::Opt(const TCHAR *Name, const TCHAR *Default) const
{
  const FString *V = Options.Find(Name);
  return V ? *V : FString(Default);
}

double ACCPhotorealTiles_C::OptNum(const TCHAR *Name, double Default) const
{
  const FString *V = Options.Find(Name);
  return (V && V->IsNumeric()) ? FCString::Atod(**V) : Default;
}

void ACCPhotorealTiles_C::BeginPlay()
{
  Super::BeginPlay();
  SpawnTime = Now();
  RunStart = FDateTime::Now().ToString(TEXT("%Y-%m-%dT%H:%M:%S"));
}

void ACCPhotorealTiles_C::Tick(float DeltaSeconds)
{
  Super::Tick(DeltaSeconds);
  if (State == EState::Waiting)
  {
    // CARLA registers the actor (with its description) right after SpawnActor returns: read role_name then
    UCarlaEpisode *Episode = UCarlaStatics::GetCurrentEpisode(this);
    FCarlaActor *CarlaActor = Episode ? Episode->FindCarlaActor(this) : nullptr;
    if (CarlaActor == nullptr && Now() - SpawnTime < 2.0)
    {
      return;
    }
    if (CarlaActor != nullptr && CarlaActor->GetActorInfo() != nullptr)
    {
      const FActorAttribute *RoleAttr = CarlaActor->GetActorInfo()->Description.Variations.Find(TEXT("role_name"));
      TArray<FString> Pairs;
      if (RoleAttr != nullptr)
      {
        RoleAttr->Value.ParseIntoArray(Pairs, TEXT(";"), true);
      }
      for (const FString &Pair : Pairs)
      {
        FString K, V;
        if (Pair.Split(TEXT("="), &K, &V))
        {
          Options.Add(K.TrimStartAndEnd().ToLower(), V.TrimStartAndEnd());
        }
      }
    }
    MapName = Episode ? Episode->GetMapName() : FString();
    if (MapName.IsEmpty())
    {
      MapName = UGameplayStatics::GetCurrentLevelName(this, true);
    }
    MapName = FPaths::GetBaseFilename(MapName);
    FString Why;
    if (!Start(Why))
    {
      UE_LOG(LogCoupledPhotoreal, Warning, TEXT("CoupledPhotoreal: photorealistic tiles off: %s; the level stays as it is."),
             *Why);
      State = EState::Stopped;
      Destroy();
      return;
    }
    State = EState::Running;
    StartTime = Now();
    return;
  }
  if (State != EState::Running)
  {
    return;
  }
  if (!PendingStop.IsEmpty())
  {
    StopTiles(PendingStop);
    return;
  }
  CheckReload();
  if (State != EState::Running)
  {
    return;
  }
  const double T = Now();
  // test only (with a local url): make Cesium reload the tileset, to check the reload count and run_cap
  const double TestRefresh = OptNum(TEXT("test_refresh_s"), 0.0);
  if (TestRefresh > 0.0 && !bTestRefreshed && !Opt(TEXT("url"), TEXT("")).IsEmpty() && T - StartTime > TestRefresh)
  {
    bTestRefreshed = true;
    UE_LOG(LogCoupledPhotoreal, Log, TEXT("CoupledPhotoreal: test: RefreshTileset"));
    Tileset->RefreshTileset();
  }
  if (T >= NextScan)
  {
    NextScan = T + 1.0;
    UpdateNight();
    UpdateHold(T);
    ScanTwinMeshes();
  }
  if (T >= NextCredits)
  {
    NextCredits = T + 10.0;
    UpdateCredits();
  }
  UpdateCameras(T);
  ApplyNativeOptions();
  if (bControl)
  {
    CheckControl(T);
  }
  UpdateProfile(T, DeltaSeconds);
  // a session lasts up to three hours: renew once before it runs out (one more billed root), if the caps allow
  const double SessionS = OptNum(TEXT("session_h"), 2.9) * 3600.0;
  if (SessionS > 0.0 && T - StartTime > SessionS)
  {
    FString Why;
    if (RootsThisRun < RunCap && LedgerAllows(Why))
    {
      UE_LOG(LogCoupledPhotoreal, Log, TEXT("CoupledPhotoreal: renewing the tile session after %.1f h (one more root request)"),
             (T - StartTime) / 3600.0);
      StartTime = T;
      Tileset->RefreshTileset();   // CheckReload counts it
    }
    else
    {
      StopTiles(Why.IsEmpty() ? FString::Printf(TEXT("the tile session ran out and run_cap (%d) is reached"), RunCap) : Why);
    }
  }
}

bool ACCPhotorealTiles_C::Start(FString &Why)
{
  RunCap = FMath::Max(1, (int32)OptNum(TEXT("run_cap"), 2.0));
  CutTop = OptNum(TEXT("cut_top"), 2.8);
  CutGrow = OptNum(TEXT("grow"), 0.0);
  bFeedCameras = Opt(TEXT("cams"), TEXT("1")) == TEXT("1");
  PlayerCameras = Opt(TEXT("player"), TEXT("auto"));
  ProfEvery = OptNum(TEXT("prof"), 0.0);
  bControl = Opt(TEXT("control"), TEXT("0")) == TEXT("1");
  bNightBuildings = Opt(TEXT("night"), TEXT("1")) == TEXT("1");
  const FString TestUrl = Opt(TEXT("url"), TEXT(""));
  const bool bTest = !TestUrl.IsEmpty();
  if (bTest && !(TestUrl.StartsWith(TEXT("http://127.0.0.1")) || TestUrl.StartsWith(TEXT("http://localhost"))))
  {
    Why = TEXT("the url option takes only a local test server (http://127.0.0.1 or http://localhost)");
    return false;
  }
  FString Key;
  if (!bTest)
  {
    FString Source;
    Key = ReadKey(Source);
    if (Key.IsEmpty())
    {
      Why = FString::Printf(TEXT("no Google Maps API key on this machine (set GOOGLE_MAPS_API_KEY or write it to %s, mode 600)"),
                            *FPaths::Combine(ConfigDir(), TEXT("google_maps_key")));
      return false;
    }
    UE_LOG(LogCoupledPhotoreal, Log, TEXT("CoupledPhotoreal: Google Maps API key found (%s)"), *Source);
  }
  if (!LedgerAllows(Why))
  {
    return false;
  }
  if (!LoadLevelData(MapName, Why))
  {
    return false;
  }
  SpawnCesium(Key);
  return Tileset != nullptr;
}

bool ACCPhotorealTiles_C::LoadLevelData(const FString &Level, FString &Why)
{
  const FString Name = Opt(TEXT("data"), *Level);
  const FString Rel = FString::Printf(TEXT("Photoreal/%s.json"), *Name);
  TArray<FString> Files = UCarlaContentPackManager::FindPackCatalogFiles(Rel);
  if (Files.Num() == 0)
  {
    Why = FString::Printf(TEXT("no %s in a content pack (python -m coupledcarla.photoreal prepare <experiment>, then start the "
                               "server with -carla-packs=<the CoupledCarla pack>)"), *Rel);
    return false;
  }
  TSharedPtr<FJsonObject> Root;
  if (!ReadJsonFile(Files.Last(), Root))
  {
    Why = FString::Printf(TEXT("cannot read %s"), *Files.Last());
    return false;
  }
  const TSharedPtr<FJsonObject> *Origin = nullptr;
  if (!Root->TryGetObjectField(TEXT("origin"), Origin))
  {
    Why = FString::Printf(TEXT("%s has no origin"), *Files.Last());
    return false;
  }
  OriginLon = Num(*Origin, TEXT("lon"));
  OriginLat = Num(*Origin, TEXT("lat"));
  OriginHeight = Num(*Origin, TEXT("height"));
  // the per-run height correction measured against the photo mesh (photoreal.height_offset_m)
  OriginHeight -= OptNum(TEXT("dz"), 0.0);
  if (Opt(TEXT("carve"), TEXT("1")) != TEXT("0"))
  {
    CarveRings = RingsFromJson(Root, TEXT("carve"));
  }
  FootprintRings = RingsFromJson(Root, TEXT("footprints"));
  const TSharedPtr<FJsonObject> *Cut = nullptr;
  const bool bWantMask = Opt(TEXT("cut"), TEXT("mask")) == TEXT("mask") || bControl;
  if (Opt(TEXT("carve"), TEXT("1")) != TEXT("0") && bWantMask && Root->TryGetObjectField(TEXT("cut"), Cut))
  {
    FString CutWhy;
    if (!LoadCutGrid(*Cut, FPaths::GetPath(Files.Last()), CutWhy))
    {
      UE_LOG(LogCoupledPhotoreal, Warning, TEXT("CoupledPhotoreal: no cut grid (%s)"), *CutWhy);
    }
  }
  const TArray<TSharedPtr<FJsonValue>> *JCov = nullptr;
  if (Root->TryGetArrayField(TEXT("coverage"), JCov))
  {
    for (const TSharedPtr<FJsonValue> &JP : *JCov)
    {
      const TArray<TSharedPtr<FJsonValue>> &P = JP->AsArray();
      if (P.Num() >= 2)
      {
        Coverage.Add(FVector2D(P[0]->AsNumber(), P[1]->AsNumber()));
      }
    }
  }
  FString Hide = Opt(TEXT("hide"), TEXT(""));
  if (Hide.IsEmpty())
  {
    const TArray<TSharedPtr<FJsonValue>> *JHide = nullptr;
    if (Root->TryGetArrayField(TEXT("hide"), JHide))
    {
      TArray<FString> Names;
      for (const TSharedPtr<FJsonValue> &V : *JHide)
      {
        Names.Add(V->AsString());
      }
      Hide = FString::Join(Names, TEXT(","));
    }
  }
  TArray<FString> Names;
  Hide.ParseIntoArray(Names, TEXT(","), true);
  for (const FString &N : Names)
  {
    if (N.TrimStartAndEnd() != TEXT("none"))
    {
      HideFolders.Add(SemanticFolder(N.TrimStartAndEnd()));
    }
  }
  UE_LOG(LogCoupledPhotoreal, Log,
         TEXT("CoupledPhotoreal: %s: origin %.7f, %.7f, %.2f m (ellipsoid), %d carve rings, %d footprints, cut grid %s, "
              "hiding %s"),
         *Files.Last(), OriginLat, OriginLon, OriginHeight, CarveRings.Num(), FootprintRings.Num(),
         bMaskReady ? TEXT("loaded") : TEXT("none"), Hide.IsEmpty() ? TEXT("nothing") : *Hide);
  return true;
}

void ACCPhotorealTiles_C::SpawnCesium(const FString &Key)
{
  UWorld *World = GetWorld();
  FActorSpawnParameters Params;
  Params.Owner = this;
  Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

  Georeference = World->SpawnActor<ACesiumGeoreference>(ACesiumGeoreference::StaticClass(), FTransform::Identity, Params);
  if (Georeference == nullptr)
  {
    UE_LOG(LogCoupledPhotoreal, Warning, TEXT("CoupledPhotoreal: cannot spawn a CesiumGeoreference"));
    return;
  }
  Georeference->SetOriginPlacement(EOriginPlacement::CartographicOrigin);
  Georeference->SetOriginLongitudeLatitudeHeight(FVector(OriginLon, OriginLat, OriginHeight));

  auto MakePolygons = [&](const TArray<TArray<FVector>> &Rings, TArray<TObjectPtr<ACesiumCartographicPolygon>> &Out)
  {
    for (const TArray<FVector> &Ring : Rings)
    {
      ACesiumCartographicPolygon *Polygon =
          World->SpawnActor<ACesiumCartographicPolygon>(ACesiumCartographicPolygon::StaticClass(), FTransform::Identity, Params);
      if (Polygon == nullptr)
      {
        continue;
      }
      Polygon->GlobeAnchor->SetGeoreference(Georeference);
      Polygon->SetPolygonPoints(ECesiumCoordinateReferenceSystem::LongitudeLatitudeHeight, Ring);
      Polygon->SetActorHiddenInGame(true);
      Out.Add(Polygon);
    }
  };
  // the cut: the mask (the tile material and the grid) when both are here, else Cesium's polygon overlay
  CutMode = Opt(TEXT("carve"), TEXT("1")) == TEXT("0") ? TEXT("none") : Opt(TEXT("cut"), TEXT("mask"));
  if (Opt(TEXT("mat"), TEXT("plugin")) == TEXT("plugin"))
  {
    TileMaterial = LoadObject<UMaterialInterface>(nullptr, TileMaterialPath, nullptr, LOAD_NoWarn);
  }
  if (CutMode == TEXT("mask") && (!bMaskReady || TileMaterial == nullptr))
  {
    UE_LOG(LogCoupledPhotoreal, Warning, TEXT("CoupledPhotoreal: cut=mask needs %s; the polygon overlay cuts instead"),
           TileMaterial == nullptr ? TEXT("the tile material (a server built with this plugin's content)")
                                   : TEXT("the level's cut grid (python -m coupledcarla.photoreal cut)"));
    CutMode = TEXT("polygons");
  }
  if (CutMode == TEXT("polygons") || bControl)
  {
    MakePolygons(CarveRings, CarvePolygons);
  }
  if (bNightBuildings)
  {
    MakePolygons(FootprintRings, FootprintPolygons);
  }

  // Deferred spawn: everything that would make Cesium destroy and reload the tileset (and so send another billed
  // root request) is set before BeginPlay, which loads it once.
  Tileset = World->SpawnActorDeferred<ACesium3DTileset>(ACesium3DTileset::StaticClass(), FTransform::Identity, this, nullptr,
                                                       ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
  if (Tileset == nullptr)
  {
    UE_LOG(LogCoupledPhotoreal, Warning, TEXT("CoupledPhotoreal: cannot spawn a Cesium3DTileset"));
    return;
  }
  const FString TestUrl = Opt(TEXT("url"), TEXT(""));
  Tileset->SetGeoreference(Georeference);
  Tileset->SetTilesetSource(ETilesetSource::FromUrl);
  const bool bKeyInUrl = Opt(TEXT("key_in"), TEXT("header")) == TEXT("url");
  if (!Key.IsEmpty() && bKeyInUrl)
  {
    // fallback if Google ever refuses the header: Cesium passes the root's query on to the tiles, but its own log
    // lines about failed requests then carry the key
    UE_LOG(LogCoupledPhotoreal, Warning, TEXT("CoupledPhotoreal: key_in=url: the key is in the request URLs (and so in "
                                              "Cesium's log lines about failed requests)"));
    Tileset->SetUrl(FString(GoogleRootUrl) + TEXT("?key=") + FGenericPlatformHttp::UrlEncode(Key));
  }
  else
  {
    Tileset->SetUrl(TestUrl.IsEmpty() ? FString(GoogleRootUrl) : TestUrl);
  }
  if (!Key.IsEmpty() && !bKeyInUrl)
  {
    // the header keeps the key out of URLs (and so out of Cesium's own log lines about requests)
    TMap<FString, FString> Headers;
    Headers.Add(TEXT("X-Goog-Api-Key"), Key);
    Tileset->SetRequestHeaders(Headers);
  }
  Tileset->SetCreatePhysicsMeshes(false);    // collision stays the level's (CARLA's roads and the hidden buildings)
  Tileset->SetCreateNavCollision(false);
  Tileset->SetIgnoreKhrMaterialsUnlit(Opt(TEXT("relit"), TEXT("0")) == TEXT("1"));
  Tileset->SetMaximumScreenSpaceError(OptNum(TEXT("sse"), 16.0));
  // tile loading (fixed before BeginPlay, so no reload and no extra root request):
  // forbid_holes: keep a parent tile until its children are in (no blank holes while refining, at a coarser look);
  // culled_sse: tiles outside the spectator's view are still loaded to this error (px, 0 = not at all), so other
  // cameras (stations, a drone, a film rig) do not look into gaps; loads / cache_mb: parallel loads, tile cache size
  Tileset->ForbidHoles = Opt(TEXT("forbid_holes"), TEXT("1")) == TEXT("1");
  const double CulledSse = OptNum(TEXT("culled_sse"), 64.0);
  Tileset->EnforceCulledScreenSpaceError = CulledSse > 0.0;
  if (CulledSse > 0.0)
  {
    Tileset->CulledScreenSpaceError = CulledSse;
  }
  Tileset->MaximumSimultaneousTileLoads = FMath::Clamp((int32)OptNum(TEXT("loads"), 32.0), 1, 128);
  Tileset->MaximumCachedBytes = (int64)FMath::Clamp(OptNum(TEXT("cache_mb"), 1024.0), 64.0, 16384.0) * 1024 * 1024;
  Tileset->ShowCreditsOnScreen = true;
  Tileset->EnableFogCulling = Opt(TEXT("fog"), TEXT("1")) == TEXT("1");
  if (TileMaterial != nullptr)
  {
    Tileset->SetMaterial(TileMaterial);   // before BeginPlay: no reload
  }
  Tileset->SetLifecycleEventReceiver(this);
  SetClip(false);
  UGameplayStatics::FinishSpawningActor(Tileset, FTransform::Identity);
  LedgerCountRoot();   // BeginPlay -> LoadTileset has sent the root request
  FailureHandle = OnCesium3DTilesetLoadFailure.AddUObject(this, &ACCPhotorealTiles_C::OnLoadFailure);
  UE_LOG(LogCoupledPhotoreal, Log, TEXT("CoupledPhotoreal: streaming %s (root requests: %d this run), cut %s, %s"),
         TestUrl.IsEmpty() ? TEXT("Google Photorealistic 3D Tiles") : *TestUrl, RootsThisRun, *CutMode,
         TileMaterial != nullptr ? TEXT("plugin tile material") : TEXT("Cesium's tile material"));
  SetViewport(Opt(TEXT("viewport"), TEXT("1")) != TEXT("0"));
}

void ACCPhotorealTiles_C::SetClip(bool bWithFootprints)
{
  if (Tileset == nullptr)
  {
    return;
  }
  if (Clip != nullptr)
  {
    Clip->DestroyComponent();   // removes its overlay from the tileset (no reload)
    Clip = nullptr;
  }
  TArray<TSoftObjectPtr<ACesiumCartographicPolygon>> Polygons;
  if (CutMode == TEXT("polygons"))
  {
    for (ACesiumCartographicPolygon *P : CarvePolygons)
    {
      Polygons.Add(P);
    }
  }
  if (bWithFootprints)
  {
    for (ACesiumCartographicPolygon *P : FootprintPolygons)
    {
      Polygons.Add(P);
    }
  }
  bClipFootprints = bWithFootprints;
  if (Polygons.Num() == 0)
  {
    return;
  }
  Clip = NewObject<UCesiumPolygonRasterOverlay>(Tileset, MakeUniqueObjectName(Tileset, UCesiumPolygonRasterOverlay::StaticClass(),
                                                                              TEXT("CCRoadClip")));
  Clip->Polygons = Polygons;
  Clip->InvertSelection = false;
  // the overlay's detail: Cesium's default (2 px error, up to 2048 px textures) rasterises every tile at up to 8x its
  // screen size, testing each pixel against each triangle of the rings over it (cesium-native)
  Clip->SetMaximumScreenSpaceError(OptNum(TEXT("clip_sse"), 2.0));
  Clip->SetMaximumTextureSize((int32)OptNum(TEXT("clip_tex"), 2048.0));
  Clip->ExcludeSelectedTiles = false;   // a tile wholly inside a road ring still loads (its kerbs may stand outside)
  Tileset->AddInstanceComponent(Clip);
  if (Tileset->HasActorBegunPlay())
  {
    Clip->RegisterComponent();   // added to the loaded tileset by Activate
  }
}

void ACCPhotorealTiles_C::ScanTwinMeshes()
{
  bool bNew = false;
  for (TActorIterator<AStaticMeshActor> It(GetWorld()); It; ++It)
  {
    AStaticMeshActor *Actor = *It;
    const FObjectKey Key(Actor);
    if (Seen.Contains(Key))
    {
      continue;
    }
    Seen.Add(Key);
    const UStaticMeshComponent *Mesh = Actor->GetStaticMeshComponent();
    if (Mesh == nullptr || Mesh->GetStaticMesh() == nullptr || Actor->IsHidden())
    {
      continue;
    }
    const FString Path = Mesh->GetStaticMesh()->GetPathName();
    if (!Path.Contains(TEXT("/Twins/")))
    {
      continue;
    }
    bool bHide = false;
    for (const FString &Folder : HideFolders)
    {
      bHide |= Path.Contains(Folder);
    }
    if (!bHide)
    {
      continue;
    }
    if (Coverage.Num() >= 3)
    {
      const FVector C = Actor->GetComponentsBoundingBox().GetCenter();
      if (!InPolygon(Coverage, FVector2D(C.X, C.Y)))
      {
        continue;
      }
    }
    Managed.Add({Actor, Path.Contains(TEXT("/Static/Building/"))});
    bNew = true;
  }
  if (bNew)
  {
    ApplyVisibility();
  }
}

void ACCPhotorealTiles_C::ApplyVisibility()
{
  int32 NHidden = 0;
  for (const FManaged &M : Managed)
  {
    if (AActor *A = M.Actor.Get())
    {
      // visibility only: collision (cars, walkers, LiDAR) and navigation keep the twin's geometry
      const bool bShow = (M.bBuilding && bNight && bNightBuildings && FootprintPolygons.Num() > 0) || !bTilesReady ||
                         !bTilesShown;
      A->SetActorHiddenInGame(!bShow);
      NHidden += bShow ? 0 : 1;
    }
  }
  if (NHidden != LastHidden && Managed.Num() > 0)
  {
    LastHidden = NHidden;
    UE_LOG(LogCoupledPhotoreal, Log, TEXT("CoupledPhotoreal: %d twin meshes hidden under the tiles (collision kept)"), NHidden);
  }
}

void ACCPhotorealTiles_C::UpdateHold(double T)
{
  const double Hold = OptNum(TEXT("hold"), 95.0);
  const double Low = OptNum(TEXT("hold_low"), 0.0);
  const double Progress = Tileset ? Tileset->GetLoadProgress() : 0.0;
  bool bReady = bTilesReady;
  if (Hold <= 0.0)
  {
    bReady = true;
  }
  else if (!bTilesReady)
  {
    // loaded this far for two scans in a row (the progress also reads 100 before the first tiles are requested),
    // or 60 s at most so a slow network cannot keep the twin up for ever
    ReadySince = Progress >= Hold ? (ReadySince < 0.0 ? T : ReadySince) : -1.0;
    bReady = (ReadySince >= 0.0 && T - ReadySince >= 1.5) || T - StartTime > OptNum(TEXT("hold_max_s"), 60.0);
  }
  else if (Low > 0.0 && Progress < Low)
  {
    bReady = false;
    ReadySince = -1.0;
  }
  if (bReady != bTilesReady)
  {
    bTilesReady = bReady;
    UE_LOG(LogCoupledPhotoreal, Log, TEXT("CoupledPhotoreal: tiles %s (load progress %.0f%%, %.1f s after start)"),
           bReady ? TEXT("loaded: the twin goes under them") : TEXT("loading: the twin shows meanwhile"), Progress,
           T - StartTime);
    ApplyVisibility();
  }
}

void ACCPhotorealTiles_C::UpdateNight()
{
  UCarlaEpisode *Episode = UCarlaStatics::GetCurrentEpisode(this);
  AWeather *Weather = Episode ? Episode->GetWeather() : nullptr;
  if (Weather == nullptr)
  {
    return;
  }
  // CARLA lights its street lamps from altitude 0 down (season.py measured on at 0, off at 1)
  const bool bNow = Weather->GetCurrentWeather().SunAltitudeAngle <= OptNum(TEXT("night_alt"), 0.0);
  if (bNightKnown && bNow == bNight)
  {
    return;
  }
  bNightKnown = true;
  bNight = bNow;
  if (bNightBuildings && FootprintPolygons.Num() > 0)
  {
    // at night the twin's own buildings (lit windows) stand in for the photo mesh's, cut out along the footprints
    SetClip(bNight);
    UE_LOG(LogCoupledPhotoreal, Log, TEXT("CoupledPhotoreal: %s: %s"), bNight ? TEXT("night") : TEXT("day"),
           bNight ? TEXT("the twin's buildings (lit windows) replace the photo mesh's") : TEXT("photo mesh buildings"));
  }
  ApplyVisibility();
}

void ACCPhotorealTiles_C::CheckReload()
{
  // Cesium destroys and reloads a tileset when some of its properties change: each reload is a new root request,
  // billed. While the root tile is missing a load is under way; a missing root after one was there is a reload.
  Cesium3DTilesSelection::Tileset *Native = Tileset ? Tileset->GetTileset() : nullptr;
  const void *RootTile = Native ? (const void *)Native->getRootTile() : nullptr;
  if (RootTile != nullptr)
  {
    NativeTileset = RootTile;
    return;
  }
  if (NativeTileset != nullptr)
  {
    NativeTileset = nullptr;
    LedgerCountRoot();
    UE_LOG(LogCoupledPhotoreal, Warning, TEXT("CoupledPhotoreal: the tileset is reloading: one more root request (%d this run)"),
           RootsThisRun);
    if (RootsThisRun > RunCap)
    {
      StopTiles(FString::Printf(TEXT("more than run_cap (%d) root requests in this run"), RunCap));
    }
  }
}

void ACCPhotorealTiles_C::UpdateCredits()
{
  ACesiumCreditSystem *Credits = ACesiumCreditSystem::GetDefaultCreditSystem(this);
  if (Credits == nullptr || !Credits->GetExternalCreditSystem())
  {
    return;
  }
  const CesiumUtility::CreditsSnapshot &Snapshot = Credits->GetExternalCreditSystem()->getSnapshot();
  TArray<FString> Parts;
  for (const CesiumUtility::Credit &Credit : Snapshot.currentCredits)
  {
    const FString Text = StripHtml(UTF8_TO_TCHAR(Credits->GetExternalCreditSystem()->getHtml(Credit).c_str()));
    if (!Text.IsEmpty())
    {
      Parts.AddUnique(Text);
    }
  }
  const FString Line = FString::Join(Parts, TEXT("; "));
  if (Line.IsEmpty() || Line == LastCredits)
  {
    return;
  }
  LastCredits = Line;
  // the attribution Google requires on rendered views; coupledcarla stamps saved frames with it (photoreal.attribution)
  UE_LOG(LogCoupledPhotoreal, Log, TEXT("CoupledPhotoreal: credits: %s"), *Line);
  FFileHelper::SaveStringToFile(Line, *FPaths::Combine(ConfigDir(), TEXT("google_tiles_credits.txt")));
}

void ACCPhotorealTiles_C::OnLoadFailure(const FCesium3DTilesetLoadFailureDetails &Details)
{
  if (Details.Tileset.Get() != Tileset || State != EState::Running)
  {
    return;
  }
  const int32 Code = Details.HttpStatusCode;
  FString Hint = TEXT("");
  if (Code == 400 || Code == 401 || Code == 403)
  {
    Hint = TEXT(": the key is refused (Map Tiles API not enabled for it, billing off, or an API/referrer restriction)");
  }
  else if (Code == 429)
  {
    Hint = TEXT(": over the project's quota");
  }
  // stopped from our own Tick, not inside Cesium's callback
  PendingStop = FString::Printf(TEXT("the root tileset did not load (HTTP %d)%s"), Code, *Hint);
}

void ACCPhotorealTiles_C::StopTiles(const FString &Why)
{
  UE_LOG(LogCoupledPhotoreal, Warning, TEXT("CoupledPhotoreal: photorealistic tiles off: %s; the twin is shown as it is."), *Why);
  State = EState::Stopped;
  if (Tileset != nullptr)
  {
    Tileset->SuspendUpdate = true;
    Tileset->SetActorHiddenInGame(true);
  }
  RestoreTwin();
  Destroy();   // the client sees the actor gone
}

void ACCPhotorealTiles_C::RestoreTwin()
{
  for (const FManaged &M : Managed)
  {
    if (AActor *A = M.Actor.Get())
    {
      A->SetActorHiddenInGame(false);
    }
  }
  Managed.Reset();
}

void ACCPhotorealTiles_C::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  if (FailureHandle.IsValid())
  {
    OnCesium3DTilesetLoadFailure.Remove(FailureHandle);
  }
  RestoreTwin();
  FlushProfile(Now());
  if (ACesiumCameraManager *Manager = ACesiumCameraManager::GetDefaultCameraManager(this))
  {
    Manager->AdditionalCameras.Reset();
    Manager->UsePlayerCameras = true;
  }
  SetViewport(true);
  if (Tileset != nullptr)
  {
    Tileset->Destroy();
  }
  for (ACesiumCartographicPolygon *P : CarvePolygons)
  {
    if (P != nullptr)
      P->Destroy();
  }
  for (ACesiumCartographicPolygon *P : FootprintPolygons)
  {
    if (P != nullptr)
      P->Destroy();
  }
  if (Georeference != nullptr)
  {
    Georeference->Destroy();
  }
  if (RootsThisRun > 0)
  {
    UE_LOG(LogCoupledPhotoreal, Log, TEXT("CoupledPhotoreal: done, %d root request(s) this run (ledger %s)"), RootsThisRun,
           *LedgerPath(!Opt(TEXT("url"), TEXT("")).IsEmpty()));
  }
  Super::EndPlay(EndPlayReason);
}

// ---------------------------------------------------------------------------------------------------------- ledger

static FString LedgerPath(bool bTest)
{
  return FPaths::Combine(ACCPhotorealTiles_C::ConfigDir(),
                         bTest ? TEXT("google_tiles_usage_test.json") : TEXT("google_tiles_usage.json"));
}

static TSharedRef<FJsonObject> LoadLedger(const FString &Path)
{
  TSharedPtr<FJsonObject> L;
  if (!ReadJsonFile(Path, L))
  {
    L = MakeShared<FJsonObject>();
  }
  const FString Month = FDateTime::Now().ToString(TEXT("%Y-%m"));
  const FString Day = FDateTime::Now().ToString(TEXT("%Y-%m-%d"));
  if (Str(L, TEXT("month")) != Month)
  {
    L->SetStringField(TEXT("month"), Month);
    L->SetNumberField(TEXT("month_roots"), 0);
  }
  if (Str(L, TEXT("day")) != Day)
  {
    L->SetStringField(TEXT("day"), Day);
    L->SetNumberField(TEXT("day_roots"), 0);
  }
  return L.ToSharedRef();
}

bool ACCPhotorealTiles_C::LedgerAllows(FString &Why) const
{
  const bool bTest = !Opt(TEXT("url"), TEXT("")).IsEmpty();
  TSharedRef<FJsonObject> L = LoadLedger(LedgerPath(bTest));
  const int32 Day = (int32)Num(L, TEXT("day_roots"));
  const int32 Month = (int32)Num(L, TEXT("month_roots"));
  const int32 DayCap = (int32)OptNum(TEXT("day_cap"), 50.0);
  const int32 MonthCap = (int32)OptNum(TEXT("month_cap"), 1000.0);
  if (Day >= DayCap)
  {
    Why = FString::Printf(TEXT("today's cap of %d root requests is reached (photoreal.day_cap)"), DayCap);
    return false;
  }
  if (Month >= MonthCap)
  {
    Why = FString::Printf(TEXT("this month's cap of %d root requests is reached (photoreal.month_cap; 1,000 a month are free)"),
                          MonthCap);
    return false;
  }
  return true;
}

void ACCPhotorealTiles_C::LedgerCountRoot()
{
  ++RootsThisRun;
  const bool bTest = !Opt(TEXT("url"), TEXT("")).IsEmpty();
  const FString Path = LedgerPath(bTest);
  TSharedRef<FJsonObject> L = LoadLedger(Path);
  for (const TCHAR *Field : {TEXT("day_roots"), TEXT("month_roots"), TEXT("total_roots")})
  {
    L->SetNumberField(Field, Num(L, Field) + 1);
  }
  TArray<TSharedPtr<FJsonValue>> Runs;
  const TArray<TSharedPtr<FJsonValue>> *Old = nullptr;
  if (L->TryGetArrayField(TEXT("runs"), Old))
  {
    Runs = *Old;
  }
  bool bFound = false;
  for (TSharedPtr<FJsonValue> &V : Runs)
  {
    TSharedPtr<FJsonObject> R = V->AsObject();
    if (R.IsValid() && Str(R, TEXT("start")) == RunStart && Str(R, TEXT("by")) == TEXT("server") &&
        Str(R, TEXT("map")) == MapName)
    {
      R->SetNumberField(TEXT("roots"), RootsThisRun);
      bFound = true;
    }
  }
  if (!bFound)
  {
    TSharedRef<FJsonObject> R = MakeShared<FJsonObject>();
    R->SetStringField(TEXT("start"), RunStart);
    R->SetStringField(TEXT("map"), MapName);
    R->SetStringField(TEXT("by"), TEXT("server"));
    R->SetNumberField(TEXT("roots"), RootsThisRun);
    Runs.Add(MakeShared<FJsonValueObject>(R));
  }
  while (Runs.Num() > MaxRunsKept)
  {
    Runs.RemoveAt(0);
  }
  L->SetArrayField(TEXT("runs"), Runs);
  if (!WriteJsonFile(Path, L))
  {
    UE_LOG(LogCoupledPhotoreal, Warning, TEXT("CoupledPhotoreal: cannot write the usage ledger %s"), *Path);
  }
}

// ------------------------------------------------------------------------------------------------ the cut grid

bool ACCPhotorealTiles_C::LoadCutGrid(const TSharedPtr<FJsonObject> &Cut, const FString &Folder, FString &Why)
{
  // written by coupledcarla.photoreal write_cut (format i8u16): zlib of an int8 plane (signed distance to the cut's
  // outline, m = v * sdf_step, negative inside) and a little-endian uint16 plane (road height, m = z_base + v / 100),
  // rows from the grid's southern edge (y0) northwards, columns from x0 eastwards (the level's metres, y north)
  if (Str(Cut, TEXT("format")) != TEXT("i8u16"))
  {
    Why = FString::Printf(TEXT("unknown cut grid format '%s'"), *Str(Cut, TEXT("format")));
    return false;
  }
  const int32 W = (int32)Num(Cut, TEXT("w")), H = (int32)Num(Cut, TEXT("h"));
  const double X0 = Num(Cut, TEXT("x0")), Y0 = Num(Cut, TEXT("y0")), Res = Num(Cut, TEXT("res"));
  const double Step = Num(Cut, TEXT("sdf_step")), ZBase = Num(Cut, TEXT("z_base"));
  if (W <= 0 || H <= 0 || W > 16384 || H > 16384 || Res <= 0.0)
  {
    Why = FString::Printf(TEXT("bad cut grid size %d x %d at %.3f m"), W, H, Res);
    return false;
  }
  const FString Path = FPaths::Combine(Folder, Str(Cut, TEXT("file")));
  TArray<uint8> Packed;
  if (!FFileHelper::LoadFileToArray(Packed, *Path))
  {
    Why = FString::Printf(TEXT("cannot read %s"), *Path);
    return false;
  }
  const int64 N = (int64)W * H;
  TArray<uint8> Raw;
  Raw.SetNumUninitialized(N * 3);
  if (!FCompression::UncompressMemory(NAME_Zlib, Raw.GetData(), (int32)Raw.Num(), Packed.GetData(), Packed.Num()))
  {
    Why = FString::Printf(TEXT("%s does not decompress to %d x %d"), *Path, W, H);
    return false;
  }
  const EPixelFormat Format = GPixelFormats[PF_G16R16F].Supported ? PF_G16R16F : PF_FloatRGBA;
  const int32 Channels = Format == PF_G16R16F ? 2 : 4;
  UTexture2D *Tex = UTexture2D::CreateTransient(W, H, Format, TEXT("CCPhotorealCutGrid"));
  if (Tex == nullptr)
  {
    Why = TEXT("cannot create the cut grid texture");
    return false;
  }
  Tex->SRGB = false;
  Tex->Filter = TF_Bilinear;
  Tex->AddressX = TA_Clamp;
  Tex->AddressY = TA_Clamp;
  Tex->NeverStream = true;
  FTexture2DMipMap &Mip = Tex->GetPlatformData()->Mips[0];
  FFloat16 *Px = static_cast<FFloat16 *>(Mip.BulkData.Lock(LOCK_READ_WRITE));
  const int8 *Sdf = reinterpret_cast<const int8 *>(Raw.GetData());
  const uint8 *Z = Raw.GetData() + N;
  for (int64 i = 0; i < N; ++i)
  {
    const uint16 Zq = (uint16)(Z[2 * i] | (Z[2 * i + 1] << 8));
    Px[Channels * i] = FFloat16((float)(Sdf[i] * Step));
    Px[Channels * i + 1] = FFloat16((float)(ZBase + Zq / 100.0));
    if (Channels == 4)
    {
      Px[Channels * i + 2] = FFloat16(0.0f);
      Px[Channels * i + 3] = FFloat16(1.0f);
    }
  }
  Mip.BulkData.Unlock();
  Tex->UpdateResource();
  CutTexture = Tex;
  // UV from Unreal world cm (x east, y south): u = (X / 100 - x0) / (w res), v = (-Y / 100 - y0) / (h res)
  CutXform = FLinearColor((float)(1.0 / (100.0 * W * Res)), (float)(-X0 / (W * Res)), (float)(-1.0 / (100.0 * H * Res)),
                          (float)(-Y0 / (H * Res)));
  bMaskReady = true;
  UE_LOG(LogCoupledPhotoreal, Log, TEXT("CoupledPhotoreal: cut grid %d x %d at %.2f m from %s"), W, H, Res, *Path);
  return true;
}

void ACCPhotorealTiles_C::ApplyCutParams(UMaterialInstanceDynamic *Mid) const
{
  if (Mid == nullptr)
  {
    return;
  }
  const bool bOn = CutMode == TEXT("mask") && bMaskReady;
  if (CutTexture != nullptr)
  {
    Mid->SetTextureParameterValue(TEXT("CCCutTex"), CutTexture);
  }
  Mid->SetVectorParameterValue(TEXT("CCCutXform"), CutXform);
  Mid->SetScalarParameterValue(TEXT("CCCutOn"), bOn ? 1.0f : 0.0f);
  Mid->SetScalarParameterValue(TEXT("CCCutTop"), CutTop > 0.0 ? (float)CutTop : 1.0e6f);   // 0: full height
  Mid->SetScalarParameterValue(TEXT("CCCutGrow"), (float)CutGrow);
}

void ACCPhotorealTiles_C::ApplyCutParamsToAll()
{
  Mids.RemoveAll([](const TWeakObjectPtr<UMaterialInstanceDynamic> &M) { return !M.IsValid(); });
  for (const TWeakObjectPtr<UMaterialInstanceDynamic> &M : Mids)
  {
    ApplyCutParams(M.Get());
  }
}

void ACCPhotorealTiles_C::SetCutMode(const FString &Mode)
{
  FString M = Mode;
  if (M == TEXT("mask") && (!bMaskReady || TileMaterial == nullptr))
  {
    UE_LOG(LogCoupledPhotoreal, Warning, TEXT("CoupledPhotoreal: cut=mask is not available here"));
    return;
  }
  if (M != TEXT("mask") && M != TEXT("polygons") && M != TEXT("none"))
  {
    return;
  }
  CutMode = M;
  SetClip(bNight && bNightBuildings && FootprintPolygons.Num() > 0);
  ApplyCutParamsToAll();
}

void ACCPhotorealTiles_C::CustomizeMaterial(ICesiumLoadedTilePrimitive &TilePrimitive, UMaterialInstanceDynamic &Material,
                                            const UCesiumMaterialUserData *CesiumData,
                                            const CesiumGltf::Material &GltfMaterial)
{
  if (TileMaterial == nullptr)
  {
    return;
  }
  if (Mids.Num() > 4 * FMath::Max(64, LoadedTiles))
  {
    Mids.RemoveAll([](const TWeakObjectPtr<UMaterialInstanceDynamic> &M) { return !M.IsValid(); });
  }
  Mids.Add(&Material);
  ApplyCutParams(&Material);
}

void ACCPhotorealTiles_C::OnTileVisibilityChanged(ICesiumLoadedTile &Tile, bool bVisible)
{
  if (bVisible)
  {
    VisibleTiles.Add(&Tile);
  }
  else
  {
    VisibleTiles.Remove(&Tile);
  }
}

void ACCPhotorealTiles_C::OnTileLoaded(ICesiumLoadedTile &Tile)
{
  ++LoadedTiles;
}

void ACCPhotorealTiles_C::OnTileUnloading(ICesiumLoadedTile &Tile)
{
  LoadedTiles = FMath::Max(0, LoadedTiles - 1);
  VisibleTiles.Remove(&Tile);
}

// ----------------------------------------------------------------------------------------- cameras and viewport

void ACCPhotorealTiles_C::UpdateCameras(double T)
{
  ACesiumCameraManager *Manager = ACesiumCameraManager::GetDefaultCameraManager(this);
  if (Manager == nullptr)
  {
    return;
  }
  if (!bFeedCameras)
  {
    if (LastCams != 0)
    {
      Manager->AdditionalCameras.Reset();
      Manager->UsePlayerCameras = PlayerCameras != TEXT("0");
      LastCams = 0;
    }
    return;
  }
  if (T >= NextCamScan)
  {
    // CARLA's camera sensors are ASensor actors holding a scene capture, which Cesium does not look for (it finds
    // ASceneCapture2D actors and player cameras): every scene capture of this world outside an ASceneCapture2D
    NextCamScan = T + 1.0;
    Captures.Reset();
    UWorld *World = GetWorld();
    for (TObjectIterator<USceneCaptureComponent2D> It; It && Captures.Num() < MaxCameras; ++It)
    {
      USceneCaptureComponent2D *C = *It;
      if (C == nullptr || C->IsTemplate() || C->GetWorld() != World || !IsValid(C->GetOwner()) ||
          C->GetOwner()->IsA<ASceneCapture2D>() || C->ProjectionType != ECameraProjectionMode::Perspective ||
          C->TextureTarget == nullptr || C->TextureTarget->SizeX < 1 || C->TextureTarget->SizeY < 1)
      {
        continue;
      }
      Captures.Add(C);
    }
  }
  TArray<FCesiumCamera> Cams;
  for (const TWeakObjectPtr<USceneCaptureComponent2D> &W : Captures)
  {
    const USceneCaptureComponent2D *C = W.Get();
    if (C == nullptr || C->TextureTarget == nullptr)
    {
      continue;
    }
    Cams.Emplace(FVector2D(C->TextureTarget->SizeX, C->TextureTarget->SizeY), C->GetComponentLocation(),
                 C->GetComponentRotation(), (double)C->FOVAngle);
  }
  Manager->AdditionalCameras = Cams;
  // the spectator (the player camera) only while no sensor looks, unless asked for (player=1) or never (player=0)
  Manager->UsePlayerCameras = PlayerCameras == TEXT("1") || (PlayerCameras == TEXT("auto") && Cams.Num() == 0);
  if (Cams.Num() != LastCams)
  {
    LastCams = Cams.Num();
    UE_LOG(LogCoupledPhotoreal, Log, TEXT("CoupledPhotoreal: tiles load for %d camera sensor(s)%s"), LastCams,
           Manager->UsePlayerCameras ? TEXT(" and the spectator") : TEXT(""));
  }
}

void ACCPhotorealTiles_C::SetViewport(bool bOn)
{
  // the server window's own view of the world (the spectator's); camera sensors capture on their own
  if (GEngine != nullptr && GEngine->GameViewport != nullptr && bViewportOff == bOn)
  {
    GEngine->GameViewport->bDisableWorldRendering = !bOn;
    bViewportOff = !bOn;
    UE_LOG(LogCoupledPhotoreal, Log, TEXT("CoupledPhotoreal: the server window %s the world"),
           bOn ? TEXT("renders") : TEXT("no longer renders"));
  }
}

void ACCPhotorealTiles_C::ApplyNativeOptions()
{
  if (bNativeOptionsApplied || Tileset == nullptr)
  {
    return;
  }
  Cesium3DTilesSelection::Tileset *Native = Tileset->GetTileset();
  if (Native == nullptr)
  {
    return;
  }
  bNativeOptionsApplied = true;
  // game-thread time per frame for turning loaded tiles into meshes (Cesium for Unreal sets 5 ms)
  const double LoadMs = OptNum(TEXT("load_ms"), 0.0);
  if (LoadMs > 0.0)
  {
    Native->getOptions().mainThreadLoadingTimeLimit = LoadMs;
    Native->getOptions().tileCacheUnloadTimeLimit = LoadMs;
  }
}

// ------------------------------------------------------------------------------------- profile and control file

void ACCPhotorealTiles_C::UpdateProfile(double T, float DeltaSeconds)
{
  if (ProfEvery <= 0.0)
  {
    return;
  }
  if (ProfStart <= 0.0)
  {
    ProfStart = ProfLastTick = T;
    NextProf = T + ProfEvery;
    return;
  }
  ++ProfFrames;
  const double Wall = FMath::Max(0.0, T - ProfLastTick);
  ProfLastTick = T;
  ProfWall += Wall;
  ProfMaxFrame = FMath::Max(ProfMaxFrame, Wall);
  ProfGame += FPlatformTime::ToMilliseconds(GGameThreadTime);
  ProfRender += FPlatformTime::ToMilliseconds(GRenderThreadTime);
  ProfRhi += FPlatformTime::ToMilliseconds(GRHIThreadTime);
  ProfGpu += FPlatformTime::ToMilliseconds(RHIGetGPUFrameCycles());
  if (T >= NextProf)
  {
    FlushProfile(T);
    NextProf = T + ProfEvery;
  }
}

void ACCPhotorealTiles_C::FlushProfile(double T)
{
  if (ProfEvery <= 0.0 || ProfFrames == 0)
  {
    return;
  }
  const FString Path = FPaths::Combine(ConfigDir(), TEXT("google_tiles_profile.csv"));
  const bool bNew = !FPaths::FileExists(Path);
  Cesium3DTilesSelection::Tileset *Native = Tileset ? Tileset->GetTileset() : nullptr;
  const double N = (double)ProfFrames;
  FString Line;
  if (bNew)
  {
    Line += TEXT("time,run,map,phase,frames,frame_ms,max_frame_ms,game_ms,render_ms,rhi_ms,gpu_ms,tiles_visible,"
                 "tiles_loaded,native_tiles,tile_mb,load_pct,cameras,cut,cut_top,sse,mids\n");
  }
  Line += FString::Printf(TEXT("%s,%s,%s,%s,%lld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%.1f,%.0f,%d,%s,%.2f,%.1f,%d\n"),
                          *FDateTime::Now().ToString(TEXT("%Y-%m-%dT%H:%M:%S")), *RunStart, *MapName, *Phase,
                          ProfFrames, 1000.0 * ProfWall / N, 1000.0 * ProfMaxFrame, ProfGame / N, ProfRender / N,
                          ProfRhi / N, ProfGpu / N, VisibleTiles.Num(), LoadedTiles,
                          Native ? (int32)Native->getNumberOfTilesLoaded() : 0,
                          Native ? Native->getTotalDataBytes() / 1.0e6 : 0.0, Tileset ? Tileset->GetLoadProgress() : 0.0,
                          FMath::Max(0, LastCams), *CutMode, CutTop, Tileset ? Tileset->MaximumScreenSpaceError : 0.0,
                          Mids.Num());
  FFileHelper::SaveStringToFile(Line, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(),
                                FILEWRITE_Append);
  ProfFrames = 0;
  ProfWall = ProfGame = ProfRender = ProfRhi = ProfGpu = ProfMaxFrame = 0.0;
}

void ACCPhotorealTiles_C::CheckControl(double T)
{
  if (T < NextControl)
  {
    return;
  }
  NextControl = T + 0.5;
  // photoreal_control.txt beside the ledger: "key=value" pairs (lines or ";"); applied in order whenever the text
  // changes; then photoreal_control.ack holds the applied text, so a script knows when a phase starts
  const FString Dir = ConfigDir();
  FString Text;
  if (!FFileHelper::LoadFileToString(Text, *FPaths::Combine(Dir, TEXT("photoreal_control.txt"))) || Text == LastControl)
  {
    return;
  }
  LastControl = Text;
  FlushProfile(T);    // the rows before belong to the phase before
  TArray<FString> Lines;
  Text.Replace(TEXT(";"), TEXT("\n")).ParseIntoArrayLines(Lines, true);
  for (const FString &L : Lines)
  {
    FString K, V;
    if (L.TrimStartAndEnd().Split(TEXT("="), &K, &V))
    {
      ApplyRuntime(K.TrimStartAndEnd(), V.TrimStartAndEnd());
    }
  }
  NextProf = T + ProfEvery;
  FFileHelper::SaveStringToFile(Text, *FPaths::Combine(Dir, TEXT("photoreal_control.ack")));
}

void ACCPhotorealTiles_C::ApplyRuntime(const FString &Key, const FString &Value)
{
  const double V = FCString::Atod(*Value);
  const bool bOn = Value == TEXT("1") || Value == TEXT("true");
  UE_LOG(LogCoupledPhotoreal, Log, TEXT("CoupledPhotoreal: control: %s=%s"), *Key, *Value);
  if (Key == TEXT("phase"))
  {
    Phase = Value.Replace(TEXT(","), TEXT(" "));
  }
  else if (Key == TEXT("cut"))
  {
    SetCutMode(Value);
  }
  else if (Key == TEXT("cut_top"))
  {
    CutTop = V;
    ApplyCutParamsToAll();
  }
  else if (Key == TEXT("grow"))
  {
    CutGrow = V;
    ApplyCutParamsToAll();
  }
  else if (Key == TEXT("tiles"))
  {
    // tiles=0: the twin alone in the same level load (the baseline without another root request)
    bTilesShown = bOn;
    if (Tileset != nullptr)
    {
      Tileset->SetActorHiddenInGame(!bOn);
      Tileset->SuspendUpdate = !bOn;
    }
    ApplyVisibility();
  }
  else if (Tileset == nullptr)
  {
    return;
  }
  else if (Key == TEXT("sse"))
  {
    Tileset->SetMaximumScreenSpaceError(V);
  }
  else if (Key == TEXT("culled_sse"))
  {
    Tileset->EnforceCulledScreenSpaceError = V > 0.0;
    if (V > 0.0)
    {
      Tileset->CulledScreenSpaceError = V;
    }
  }
  else if (Key == TEXT("forbid_holes"))
  {
    Tileset->ForbidHoles = bOn;
  }
  else if (Key == TEXT("loads"))
  {
    Tileset->MaximumSimultaneousTileLoads = FMath::Clamp((int32)V, 1, 128);
  }
  else if (Key == TEXT("cache_mb"))
  {
    Tileset->MaximumCachedBytes = (int64)FMath::Clamp(V, 64.0, 16384.0) * 1024 * 1024;
  }
  else if (Key == TEXT("fog"))
  {
    Tileset->EnableFogCulling = bOn;
  }
  else if (Key == TEXT("occlusion"))
  {
    Tileset->SetEnableOcclusionCulling(bOn);
  }
  else if (Key == TEXT("load_ms") && Tileset->GetTileset() != nullptr && V > 0.0)
  {
    Tileset->GetTileset()->getOptions().mainThreadLoadingTimeLimit = V;
    Tileset->GetTileset()->getOptions().tileCacheUnloadTimeLimit = V;
  }
  else if ((Key == TEXT("clip_sse") || Key == TEXT("clip_tex")) && Clip != nullptr)
  {
    if (Key == TEXT("clip_sse"))
    {
      Clip->SetMaximumScreenSpaceError(V);
    }
    else
    {
      Clip->SetMaximumTextureSize((int32)V);
    }
    Clip->Refresh();   // re-adds the overlay (no tileset reload, no root request)
  }
  else if (Key == TEXT("cams"))
  {
    bFeedCameras = bOn;
    NextCamScan = 0.0;
  }
  else if (Key == TEXT("player"))
  {
    PlayerCameras = Value;
  }
  else if (Key == TEXT("viewport"))
  {
    SetViewport(bOn);
  }
  else if (Key.StartsWith(TEXT("cvar.")))
  {
    if (IConsoleVariable *Var = IConsoleManager::Get().FindConsoleVariable(*Key.Mid(5)))
    {
      Var->Set(*Value, ECVF_SetByCode);
    }
  }
}
