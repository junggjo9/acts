set(unittest_extra_libraries ActsExamplesCudaMuonSegmentFitting)
add_unittest(CudaMuonSpacePoint CudaMuonSpacePointTests.cpp)
add_unittest(CudaHoughTransformUtils CudaHoughTransformUtilsTests.cpp)

find_package(CUDAToolkit REQUIRED)

target_link_libraries(
    ActsUnitTestCudaHoughTransformUtils
    PRIVATE
        ActsExamplesFramework
        ActsExamplesIoRoot
        CUDA::cudart
        ROOT::TreePlayer
        ROOT::Tree
        ROOT::RIO
        ROOT::Core
        ROOT::Hist
)
