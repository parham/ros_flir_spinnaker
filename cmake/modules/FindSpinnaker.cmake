
unset(Spinnaker_FOUND)
unset(Spinnaker_INCLUDE_DIRS)
unset(Spinnaker_LIBRARIES)

find_path(Spinnaker_INCLUDE_DIRS NAMES
  Spinnaker.h
  PATHS
  /usr/include/spinnaker/
)

find_library(Spinnaker_LIBRARIES NAMES Spinnaker
  PATHS
  /usr/lib/
)

if (Spinnaker_INCLUDE_DIRS AND Spinnaker_LIBRARIES)
  set(Spinnaker_FOUND 1)
  set(Spinnaker_LIBS Spinnaker)
endif (Spinnaker_INCLUDE_DIRS AND Spinnaker_LIBRARIES)
