cmake_minimum_required( VERSION 2.6 )

SET_PROPERTY(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS 1)

SUBDIRS(
  apriltagsAgentComp
  #faulhaberComp
  inversekinematicsAgent
  inversekinematicsComp
  inversekinematicsTesterComp
  #joystickArmComp
  missionAgent
  modelRenderer
  #ursusHeadComp
)