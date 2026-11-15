// Coupled CARLA patch for Cesium for Unreal on CARLA's Unreal Engine fork, Linux only.
// Copy into CesiumForUnreal/Source/CesiumRuntime/Private/ (unreal/README.md).
//
// CARLA's engine fork compiles every module with RTTI (TargetRules.bForceEnableRTTI defaults to true there), while
// Cesium for Unreal's prebuilt cesium-native libraries are built without it. A CesiumRuntime class deriving from a
// cesium-native class whose typeinfo belongs with an out-of-line (key) virtual function then refers to a typeinfo the
// libraries never emitted, and libUnrealEditor-CesiumRuntime.so fails to link:
//   undefined symbol: typeinfo for Cesium3DTilesSelection::TileOcclusionRendererProxyPool
// It is defined here exactly as a compiler would (a __class_type_info: the class has no bases). Weak, so a future
// cesium-native built with RTTI wins. MSVC (Windows) emits type descriptors on demand and needs nothing.
#if PLATFORM_LINUX && defined(__GXX_RTTI)

extern "C" void *_ZTVN10__cxxabiv117__class_type_infoE[];   // vtable for __cxxabiv1::__class_type_info

namespace CesiumCarlaRtti
{
struct FTypeInfo
{
  const void *VTable;
  const char *Name;
};
} // namespace CesiumCarlaRtti

extern const CesiumCarlaRtti::FTypeInfo CesiumCarlaTypeInfoTileOcclusionRendererProxyPool
    __asm__("_ZTIN22Cesium3DTilesSelection30TileOcclusionRendererProxyPoolE");
__attribute__((weak, visibility("default"))) const CesiumCarlaRtti::FTypeInfo
    CesiumCarlaTypeInfoTileOcclusionRendererProxyPool = {
        &_ZTVN10__cxxabiv117__class_type_infoE[2],
        "N22Cesium3DTilesSelection30TileOcclusionRendererProxyPoolE"};

#endif
