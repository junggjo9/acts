find_package(CUDAToolkit REQUIRED)

set(unittest_extra_libraries
    Acts::ExamplesCudaMuonSegmentFitting
    Acts::ExamplesFramework
    ActsExamplesIoRoot
    Acts::ExamplesTrackFinding
    CUDA::cudart
    ROOT::TreePlayer
    ROOT::Tree
    ROOT::RIO
    ROOT::Core
)

add_unittest(CudaMuonSpacePoint CudaMuonSpacePointTests.cpp)
add_unittest(EtaHoughTransform EtaHoughTransformTests.cpp)
add_unittest(PhiHoughTransform PhiHoughTransformTests.cpp)
add_unittest(EtaHoughTransformBenchmarks EtaHoughTransformBenchmarks.cpp)
add_unittest(MuonHoughTransformBenchmarks MuonHoughTransformBenchmarks.cpp)
add_unittest(CudaMuonHoughMaximum CudaMuonHoughMaximumTests.cpp)
