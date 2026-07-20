// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <boost/test/unit_test.hpp>

#include "ActsExamples/Utilities/CudaHoughTransformUtils.hpp"

namespace ActsTests {

namespace CudaHT = ActsExamples::CudaHoughTransformUtils;

BOOST_AUTO_TEST_SUITE(CudaHoughTransformUtilsSuite)


// This checks the batched cell model.
//
// We fill a cell in bucket 1 with four hit contributions:
//   layers: 1, 2, 2, 4
//
// nHits counts all contributions and becomes 4.
// nLayers counts unique layers and becomes 3.
BOOST_AUTO_TEST_CASE(cuda_hough_batch_bit_mask_layer_counting) {
  CudaHT::CudaHoughPlaneBatch batch{{4, 4}, 3};

  batch.fillBin(1, 1, 2, 1, 1.0f);
  batch.fillBin(1, 1, 2, 2, 1.0f);
  batch.fillBin(1, 1, 2, 2, 1.0f);
  batch.fillBin(1, 1, 2, 4, 1.0f);

  BOOST_CHECK_EQUAL(batch.nHits(1, 1, 2), 4.0f);
  BOOST_CHECK_EQUAL(batch.nLayers(1, 1, 2), 3.0f);

  BOOST_CHECK(batch.hasLayer(1, 1, 2, 1));
  BOOST_CHECK(batch.hasLayer(1, 1, 2, 2));
  BOOST_CHECK(batch.hasLayer(1, 1, 2, 4));
  BOOST_CHECK(!batch.hasLayer(1, 1, 2, 3));

  BOOST_CHECK_EQUAL(batch.nHits(0, 1, 2), 0.0f);
  BOOST_CHECK_EQUAL(batch.nHits(2, 1, 2), 0.0f);
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace ActsTests
