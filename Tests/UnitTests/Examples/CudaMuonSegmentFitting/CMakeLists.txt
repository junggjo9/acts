find_package(CUDAToolkit REQUIRED)

set(unittest_extra_libraries
    Acts::ExamplesCudaMuonSegmentFitting
    Acts::ExamplesFramework
    Acts::ExamplesIoRoot
    CUDA::cudart
    ROOT::TreePlayer
    ROOT::Tree
    ROOT::RIO
    ROOT::Core
    ROOT::Hist
)

add_unittest(CudaMuonSpacePoint CudaMuonSpacePointTests.cpp)
add_unittest(CudaHoughTransformUtils CudaHoughTransformUtilsTests.cpp)
add_unittest(EtaHoughTransform EtaHoughTransformTests.cpp)
add_unittest(CudaMuonHoughMaximum CudaMuonHoughMaximumTests.cpp)
