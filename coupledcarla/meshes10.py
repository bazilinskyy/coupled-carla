"""scenery.py's mesh choices for CARLA 1.0 (Unreal Engine 5.8, data/meshes_1.0.json).

CARLA 1.0 moved and renamed most of the 0.9.16 meshes scenery.py places: trees and bushes went to
Vegetation/00_LegacyAssets (SM_Maple_M_v1 is SM_Maple01_M_A there, the same mesh) or are new UE5 trees in
Vegetation/Trees (sugar maple, cypress, palms); lamps went to Static/Streetlights; benches, bins and planters to
Static and Static/00_LegacyAssets; buildings to Building/Houses, Building/Buildings/BuildingComplete (SM_Apt*,
SM_Skysc*) and Building/SpecialBuildings under new names. Only the ground tiles kept their names. These tables use
the same keys as the 0.9.16 ones in scenery.py, so the placement code is shared. A name that exists twice (Static
and Static/00_LegacyAssets) is given by its path under Content/Carla/Static.
"""
from types import SimpleNamespace

from . import scenery

# whole buildings by kind: 0.9.16's choices where 1.0 has the same mesh (same size to 5 cm: SM_House07_A is
# SM_TerracedHouse02_1, SM_Apt20_C is SM_Apartment04_v02, SM_Skysc04_B is SM_Skycraper04_2, SM_Apt18_A is
# SM_Mall02), plus 1.0's new ones that a ray test finds closed from all sides (tools: rays from 8 sides at 3
# heights and from above all hit the mesh): the narrow town houses SM_Apt32-35, the SM_House17/18 houses, the
# SM_Apt22-31 blocks, Skysc08-17 towers and the university buildings
BUILDINGS = {
    "house": ["SM_House01", "SM_House02", "SM_House03", "SM_House04", "SM_House05_B", "SM_House06", "SM_House10",
              "SM_House11", "SM_House12_A", "SM_House_13", "SM_House16", "SM_House07_A", "SM_Mansion01",
              "SM_Mansion02", "SM_Mansion03", "SM_Mansion04", "SM_House14_C", "SM_House15_A",
              "SM_House17_A", "SM_House17_B", "SM_House17_C", "SM_House18_A", "SM_House18_B", "SM_House18_C",
              "SM_House18_E", "SM_House18_F"],
    "apartments": ["SM_Apt20_A", "SM_Apt20_B", "SM_Apt20_C", "SM_Apt20_E", "SM_Apt20_F", "SM_Apt19_A", "SM_Apt12_A",
                   "SM_Apt12_B", "SM_Apt21_A", "SM_Apt01_C", "SM_Apt04_B", "SM_Apt05_A",
                   "SM_Apt32_A", "SM_Apt33_A", "SM_Apt34_A", "SM_Apt35_A", "SM_Apt11", "SM_Apt13", "SM_Apt14",
                   "SM_Apt22", "SM_Apt23", "SM_Apt24_A", "SM_Apt24_B", "SM_Apt24_C", "SM_Apt25_A", "SM_Apt25_B",
                   "SM_Apt25_C", "SM_Apt25_F", "SM_Apt25_G", "SM_Apt25_M", "SM_Apt26_A", "SM_Apt26_C", "SM_Apt26_F",
                   "SM_Apt27_A", "SM_Apt27_B", "SM_Apt27_C", "SM_Apt28_A", "SM_Apt28_B", "SM_Apt28_C", "SM_Apt29_A",
                   "SM_Apt29_B", "SM_Apt29_C", "SM_Apt30_A", "SM_Apt30_B", "SM_Apt30_C", "SM_Apt31_A", "SM_Apt31_B",
                   "SM_Apt31_C"],
    "commercial": ["SM_Apt18_A", "SM_Apt18_B", "SM_Apt18_C", "SM_Apt21_A", "SM_Apt12_B", "SM_Apt20_F",
                   "SM_medicineBuilding01", "SM_medicineBuilding02", "SM_medicineBuilding03", "SM_HumanHall",
                   "SM_CvC", "SM_Museum01", "SM_Museum02"],
    "tower": ["SM_Skysc01", "SM_Skysc02", "SM_Skysc03", "SM_Skysc05", "SM_Skysc06", "SM_Skysc04_A", "SM_Skysc04_B",
              "SM_Skysc08", "SM_Skysc10", "SM_Skysc11", "SM_Skysc12", "SM_Skysc13_A", "SM_Skysc13_B",
              "SM_Skysc14_A", "SM_Skysc14_B", "SM_Skysc14_C", "SM_Skysc15_A", "SM_Skysc15_B", "SM_Skysc16_A",
              "SM_Skysc16_B", "SM_Skysc16_C", "SM_Skysc17_A", "SM_Skysc17_B"],
    "industrial": ["SM_Factory01", "SM_Factory02", "SM_Factory03", "SM_Parking", "SM_Garage"],
    "church": ["SM_Church"],
}
# 0.9.16's BROKEN as they are named on 1.0, and what the ray test found open (rays from some sides miss: a facade
# or shell) or without collision and with an odd pivot (SM_Apt15_A)
BROKEN = {"SM_Apt04_A", "SM_Apt07_A", "SM_Apt08_A", "SM_ParkingRamp", "SM_Apt15_A", "SM_Apt18_E", "SM_Skysc07",
          "SM_Skysc09", "SM_Skysc13_C", "SM_Skysc04_C", "SM_GasStation", "SM_Construction"}
# a building whose window glass is a separate mesh with the same pivot (as the towns place them): placed with it
COMPANIONS = {f"SM_{n}": [f"SM_{n}_Glass"] for n in ("Apt20_A", "Apt20_B", "Apt20_C", "Apt20_E", "Apt20_F",
                                                    "Apt32_A", "Apt33_A", "Apt34_A", "Apt35_A")}

TREES = {   # family -> meshes, roughly small to large: 0.9.16's trees (its vN is letter N of A-H here), not the new
            # UE5 trees of Vegetation/Trees (millions of triangles, minutes to build on an uncooked server)
    "maple": ["SM_Maple01_S_D", "SM_Maple01_S_F", "SM_Maple01_M_A", "SM_Maple01_M_D", "SM_Maple01_M_E",
              "SM_Maple01_M_F", "SM_Maple01_L_A", "SM_Maple01_L_D", "SM_Maple01_L_E", "SM_Maple01_L_F"],
    "oak": ["SM_Oak01_S_A", "SM_Oak01_S_D", "SM_Oak01_S_F", "SM_Oak01_M_A", "SM_Oak01_M_D", "SM_Oak01_M_F",
            "SM_Oak01_L_A", "SM_Oak01_L_D", "SM_Oak01_L_F", "SM_QuercusRubra01_A", "SM_QuercusRubra01_B",
            "SM_QuercusRubra01_C", "SM_QuercusRubra01_D"],
    "pine": ["SM_Pine01_S_D", "SM_Pine01_M_D", "SM_Pine01_M_E", "SM_Pine01_L_D", "SM_Pine01_L_E",
             "SM_TreePine01_Lod1", "SM_TreePine02_Lod1"],
    "ash": ["SM_Ash01_A", "SM_Ash01_B", "SM_WhiteAsh01_A", "SM_WhiteAsh01_B", "SM_WhiteAsh02_A", "SM_WhiteAsh02_B"],
    "beech": ["SM_Beech01_A"],
    "walnut": ["SM_Wallnut01_A", "SM_Wallnut01_B"],
    "cypress": ["SM_Cypress01_S_A", "SM_Cypress01_L_A"],
    "palm": ["SM_PalmTree01_A", "SM_PalmTree01_B", "SM_PalmTree02_A", "SM_PalmTree02_B", "SM_PalmTree02_C"],
    "ornamental": ["SM_Acer01_A", "SM_Acer01_B", "SM_Acer02_A", "SM_JapaneseMaple01_A", "SM_JapaneseMaple01_B",
                   "SM_JapaneseMaple01_C", "SM_JapaneseMaple01_D"],
    "sassafras": ["SM_Sassafras01_A", "SM_Sassafras01_B", "SM_Sassafras01_C"],
    "tropical": ["SM_Aporosa01_A"],
    "plane": ["SM_PlatanusBase01_A", "SM_PlatanusBase01_B", "SM_PlatanusBase01_C"],
}
GENUS = dict(scenery.GENUS, platanus="plane")      # 1.0 has plane trees (0.9.16 used maples for them)
BUSHES = ["SM_Bush03_M_A", "SM_Bush03_M_C", "SM_Bush03_L_A", "SM_Bush03_L_C", "SM_oldBush03", "SM_longLeavesBush02"]

FURNITURE = {   # (OSM key, value) -> meshes
    ("amenity", "bench"): ["SM_Bench01", "Static/SM_Bench02"],
    ("leisure", "picnic_table"): ["Static/00_LegacyAssets/SM_Bench02"],
    ("amenity", "waste_basket"): ["SM_TrashCan01", "SM_TrashCan02", "SM_TrashCan03", "SM_trashCan04"],
    ("amenity", "waste_disposal"): ["SM_Dumpster"],
    ("amenity", "recycling"): ["SM_RecycleGlass", "SM_Recycle_Plastic", "SM_Dumpster02"],
    ("amenity", "post_box"): ["SM_MailBox"],
    ("amenity", "bicycle_parking"): ["SM_BikeParking01", "SM_BikeParking02"],
    ("amenity", "fountain"): ["SM_fountainSmall", "SM_Fountain_03"],
    ("amenity", "drinking_water"): ["SM_StreetFountain"],
    ("amenity", "vending_machine"): ["SM_CarlaCola"],
    ("amenity", "atm"): ["SM_Atm"],
    ("amenity", "telephone"): ["SM_LightBox1"],
    ("amenity", "shelter"): ["SM_BusStop"],
    ("barrier", "bollard"): ["SM_Bollard_Grey", "SM_Bollard_Green"],
    ("barrier", "planter"): ["SM_PlantpotM2", "SM_PlantPot06", "SM_PlanterGrass01"],
    ("emergency", "fire_hydrant"): ["SM_FireHdrant"],
    ("advertising", "column"): ["SM_Billboard1"],
    ("advertising", "billboard"): ["SM_StreetAD01"],
    ("advertising", "board"): ["SM_StreetAD02", "SM_StreetAD03"],
    ("advertising", "poster_box"): ["SM_AdvertisingPost"],
    ("man_made", "street_cabinet"): ["SM_ElectricBox01", "SM_ElectricBox02"],
    ("information", "board"): ["SM_MapTable"],
    ("playground", "swing"): ["SM_Swing"],
}
SHELTER = ["SM_BusStop"]
LAMP, PARK_LAMP, LAMP_YAW = "SM_StreetLight06_A", "SM_StreetLight03", 0.0   # 0.9.16's lamp; a 4 m park lamp
POST = "SM_Pole01"               # the same 2.3 m post as 0.9.16's SM_RoadSigns01
STOP, GIVE_WAY, SPEED, SIGN_YAW = "SM_Stop01", "SM_Yield01", "SM_SpeedLimit{}", 0.0     # 0.9.16's plates, renamed
SPEEDS = [30, 40, 50, 60, 70, 80, 90, 100, 110, 120]
GROUND_MESH = dict(scenery.GROUND_MESH, water="SM_Lake01")     # SM_TileLake is SM_Lake01 on 1.0

MESHES = SimpleNamespace(
    BUILDINGS=BUILDINGS, BROKEN=BROKEN, ROOF_OF=scenery.ROOF_OF, TREES=TREES, GENUS=GENUS, BUSHES=BUSHES,
    FURNITURE=FURNITURE, SHELTER=SHELTER, LAMP=LAMP, PARK_LAMP=PARK_LAMP, POST=POST, STOP=STOP, GIVE_WAY=GIVE_WAY,
    SPEED=SPEED, SPEEDS=SPEEDS, SIGN_YAW=SIGN_YAW, LAMP_YAW=LAMP_YAW, COMPANIONS=COMPANIONS, GROUND_PIVOT=True,
    GROUND_MESH=GROUND_MESH, PAVING=scenery.PAVING)
