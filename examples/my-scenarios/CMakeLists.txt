build_example(
  NAME my-first-scenario
  SOURCE_FILES my-first-scenario.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libpoint-to-point}
    ${libapplications}
)

build_example(
  NAME 4G-simulation-file
  SOURCE_FILES 4G_simulation_file.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${liblte}
    ${libpoint-to-point}
    ${libapplications}
    ${libflow-monitor}
)

build_example(
  NAME wifi-mesh-simulation-file
  SOURCE_FILES wifi_mesh_simulation_file.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libmesh}
    ${libwifi}
    ${libpoint-to-point}
    ${libapplications}
    ${libcsma}
    ${libbuildings}
    ${libflow-monitor}
)

build_example(
  NAME wifi-hybrid-try-2
  SOURCE_FILES wifi_hybrid_try_2.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libmesh}
    ${libwifi}
    ${libpoint-to-point}
    ${libapplications}
    ${libcsma}
    ${libbuildings}
    ${libflow-monitor}
)

build_example(
  NAME 5G-simulation-file
  SOURCE_FILES 5G_simulation_file.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libpoint-to-point}
    ${libapplications}
    ${libbuildings}
    ${libmobility}
    ${libflow-monitor}
    ${libnr}
    ${libnr-epc}
)

build_example(
  NAME wifi-hybrid-scenario
  SOURCE_FILES wifi_hybrid_scenario.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libwifi}
    ${libmesh}
    ${liblte}
    ${libnr}
    ${libbuildings}
    ${libmobility}
    ${libpoint-to-point}
    ${libapplications}
    ${libflow-monitor}
)
