include(FetchContent)
FetchContent_Declare(
  kiss-icp
  GIT_REPOSITORY https://github.com/PRBonn/kiss-icp.git
  GIT_TAG v0.2.10
  SOURCE_SUBDIR cpp/kiss_icp
)

FetchContent_MakeAvailable(kiss-icp)
