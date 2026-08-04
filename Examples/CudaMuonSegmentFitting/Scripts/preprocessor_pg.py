# This file is part of the ACTS project.
#
# Copyright (C) 2016 CERN for the benefit of the ACTS project
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

"""Prepare Particle Gun data (or generaly any) for Eta Hough validation.

The source ROOT file stores detector space points and muon truth segments in
separate trees, without a direct segment-to-bucket link. This script rebuilds
that relation by combining geometry-ID overlap with drift-radius compatibility.
It writes a compact ROOT input for the C++ validation test and, by default,
four CSV files for preprocessor rejected segments analysis.

"""

import argparse
import csv
from collections import Counter
from contextlib import nullcontext
from pathlib import Path

import awkward as ak
import numpy as np
import uproot
from numba import njit


# Only load the source branches used by this conversion.
SPACE_BRANCHES = [
    "event_id",
    "spacePoint_bucketId",
    "spacePoint_geometryId",
    "spacePoint_muonId",
    "spacePoint_localPosX",
    "spacePoint_localPosY",
    "spacePoint_localPosZ",
    "spacePoint_sensorDirPhi",
    "spacePoint_sensorDirTheta",
    "spacePoint_toNextDirPhi",
    "spacePoint_toNextDirTheta",
    "spacePoint_driftRadius",
    "spacePoint_time",
    "spacePoint_covLoc0",
    "spacePoint_covLoc1",
    "spacePoint_covT",
]

TRUTH_BRANCHES = [
    "event_id",
    "Segments_hitGeoIds",
    "Segments_localSegPars",
]

# The preprocessed output uses one row per hit and repeats its bucket metadata.
OUTPUT_SCHEMA = {
    "validation_bucket_id": "uint32",
    "event_id": "uint32",
    "source_bucket_id": "uint16",
    "n_input_hits": "uint32",
    "n_truth_segments": "uint16",
    "geometry_id": "uint64",
    "muon_id": "uint32",
    "local_pos_x": "float32",
    "local_pos_y": "float32",
    "local_pos_z": "float32",
    "sensor_dir_phi": "float32",
    "sensor_dir_theta": "float32",
    "to_next_dir_phi": "float32",
    "to_next_dir_theta": "float32",
    "drift_radius": "float32",
    "time": "float32",
    "cov_loc0": "float32",
    "cov_loc1": "float32",
    "cov_t": "float32",
}

# Truth is stored separately: one row per matched segment, with variable-length
# bucket-local hit indices linking the segment back to EtaValidationInput.
TRUTH_OUTPUT_SCHEMA = {
    "validation_truth_id": "uint32",
    "validation_bucket_id": "uint32",
    "event_id": "uint32",
    "source_bucket_id": "uint16",
    "segment_index": "uint32",
    "n_truth_hits": "uint32",
    "true_tanBeta": "float64",
    "true_y0": "float64",
    "truth_hit_indices": "var * uint32",
}


SEGMENT_DIAGNOSTIC_FIELDS = [
    "event_id",
    "segment_index",
    "failure_reason",
    "true_tanBeta",
    "true_y0",
    "n_segment_geometry_ids",
    "segment_geometry_ids",
    "n_geometry_candidate_buckets",
    "best_source_bucket_id",
    "best_n_input_hits",
    "best_geometry_overlap",
    "best_n_compatible_hits",
    "best_truth_fraction",
    "best_mean_residual_mm",
]

CANDIDATE_DIAGNOSTIC_FIELDS = [
    "event_id",
    "segment_index",
    "candidate_rank",
    "source_bucket_id",
    "n_input_hits",
    "geometry_overlap",
    "overlapping_geometry_ids",
    "n_compatible_hits",
    "truth_fraction",
    "mean_residual_mm",
    "passes_min_truth_hits",
]

HIT_DIAGNOSTIC_FIELDS = [
    "event_id",
    "segment_index",
    "source_bucket_id",
    "bucket_local_hit_index",
    "geometry_id",
    "geometry_matches_segment",
    "muon_id",
    "logical_layer",
    "local_pos_x",
    "local_pos_y",
    "local_pos_z",
    "drift_radius",
    "cov_loc1",
    "residual_mm",
    "truth_compatible",
]

PEER_SEGMENT_DIAGNOSTIC_FIELDS = [
    "event_id",
    "target_segment_index",
    "source_bucket_id",
    "peer_segment_index",
    "peer_true_tanBeta",
    "peer_true_y0",
    "peer_n_geometry_ids",
    "peer_geometry_ids",
    "geometry_overlap",
    "overlapping_geometry_ids",
]


def candidate_sort_key(candidate):
    """Return the priority used to choose one bucket for a truth segment."""
    return (
        candidate["n_truth_hits"],
        candidate["truth_fraction"],
        candidate["geometry_overlap"],
        -candidate["mean_residual"],
    )


class UnmatchedDiagnostics:
    """Own and fill the four related unmatched-segment diagnostic CSV files."""

    def __init__(self, prefix, minimum_truth_hits):
        prefix.parent.mkdir(parents=True, exist_ok=True)
        self.minimum_truth_hits = minimum_truth_hits
        self.segment_path = prefix.with_name(
            prefix.name + "_segments.csv"
        )
        self.candidate_path = prefix.with_name(
            prefix.name + "_candidates.csv"
        )
        self.hit_path = prefix.with_name(prefix.name + "_hits.csv")
        self.peer_segment_path = prefix.with_name(
            prefix.name + "_peer_segments.csv"
        )

        self.segment_file = self.segment_path.open("w", newline="")
        self.candidate_file = self.candidate_path.open("w", newline="")
        self.hit_file = self.hit_path.open("w", newline="")
        self.peer_segment_file = self.peer_segment_path.open(
            "w", newline=""
        )

        self.segment_writer = csv.DictWriter(
            self.segment_file, fieldnames=SEGMENT_DIAGNOSTIC_FIELDS
        )
        self.candidate_writer = csv.DictWriter(
            self.candidate_file,
            fieldnames=CANDIDATE_DIAGNOSTIC_FIELDS,
        )
        self.hit_writer = csv.DictWriter(
            self.hit_file, fieldnames=HIT_DIAGNOSTIC_FIELDS
        )
        self.peer_segment_writer = csv.DictWriter(
            self.peer_segment_file,
            fieldnames=PEER_SEGMENT_DIAGNOSTIC_FIELDS,
        )
        self.segment_writer.writeheader()
        self.candidate_writer.writeheader()
        self.hit_writer.writeheader()
        self.peer_segment_writer.writeheader()

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception, traceback):
        self.segment_file.close()
        self.candidate_file.close()
        self.hit_file.close()
        self.peer_segment_file.close()

    def write_segment(
        self,
        event_id,
        segment_index,
        failure_reason,
        tan_beta,
        y0,
        segment_geometry_set,
        candidates,
        event_arrays,
        event_segments,
    ):
        """Write one failed segment and all of its possible associations."""

        # 1. Rank every geometry-overlapping bucket, even when it failed the
        # minimum-compatible-hit requirement used by the main preprocessor.
        ordered = sorted(
            candidates, key=candidate_sort_key, reverse=True
        )
        best = ordered[0] if ordered else None

        # 2. Write one compact row explaining the segment-level failure.
        self.segment_writer.writerow({
            "event_id": event_id,
            "segment_index": segment_index,
            "failure_reason": failure_reason,
            "true_tanBeta": "" if tan_beta is None else tan_beta,
            "true_y0": "" if y0 is None else y0,
            "n_segment_geometry_ids": len(segment_geometry_set),
            "segment_geometry_ids": ";".join(
                f"0x{geometry_id:x}"
                for geometry_id in sorted(segment_geometry_set)
            ),
            "n_geometry_candidate_buckets": len(ordered),
            "best_source_bucket_id": (
                "" if best is None else best["source_bucket_id"]
            ),
            "best_n_input_hits": (
                "" if best is None else len(best["indices"])
            ),
            "best_geometry_overlap": (
                "" if best is None else best["geometry_overlap"]
            ),
            "best_n_compatible_hits": (
                "" if best is None else best["n_truth_hits"]
            ),
            "best_truth_fraction": (
                "" if best is None else best["truth_fraction"]
            ),
            "best_mean_residual_mm": (
                "" if best is None else best["mean_residual"]
            ),
        })

        # 3. Expand each candidate into a bucket row and its hit-level rows.
        for candidate_rank, candidate in enumerate(ordered):
            source_bucket_id = candidate["source_bucket_id"]
            indices = candidate["indices"]
            bucket_geometry_set = set(map(
                int,
                np.unique(event_arrays["geometry_id"][indices]),
            ))
            overlapping_geometry_ids = sorted(
                segment_geometry_set & bucket_geometry_set
            )
            self.candidate_writer.writerow({
                "event_id": event_id,
                "segment_index": segment_index,
                "candidate_rank": candidate_rank,
                "source_bucket_id": source_bucket_id,
                "n_input_hits": len(indices),
                "geometry_overlap": candidate["geometry_overlap"],
                "overlapping_geometry_ids": ";".join(
                    f"0x{geometry_id:x}"
                    for geometry_id in overlapping_geometry_ids
                ),
                "n_compatible_hits": candidate["n_truth_hits"],
                "truth_fraction": candidate["truth_fraction"],
                "mean_residual_mm": candidate["mean_residual"],
                "passes_min_truth_hits": int(
                    candidate["n_truth_hits"]
                    >= self.minimum_truth_hits
                ),
            })

            for local_hit_index, event_hit_index in enumerate(indices):
                geometry_id = int(
                    event_arrays["geometry_id"][event_hit_index]
                )
                muon_id = int(event_arrays["muon_id"][event_hit_index])
                self.hit_writer.writerow({
                    "event_id": event_id,
                    "segment_index": segment_index,
                    "source_bucket_id": source_bucket_id,
                    "bucket_local_hit_index": local_hit_index,
                    "geometry_id": geometry_id,
                    "geometry_matches_segment": int(
                        geometry_id in segment_geometry_set
                    ),
                    "muon_id": muon_id,
                    "logical_layer": (muon_id >> 17) & 0xF,
                    "local_pos_x": event_arrays["local_x"][
                        event_hit_index
                    ],
                    "local_pos_y": event_arrays["local_y"][
                        event_hit_index
                    ],
                    "local_pos_z": event_arrays["local_z"][
                        event_hit_index
                    ],
                    "drift_radius": event_arrays["drift_radius"][
                        event_hit_index
                    ],
                    "cov_loc1": event_arrays["cov1"][event_hit_index],
                    "residual_mm": candidate["residuals"][
                        local_hit_index
                    ],
                    "truth_compatible": int(
                        candidate["truth_mask"][local_hit_index]
                    ),
                })

            # 4. Record other muon truth segments sharing geometry with this
            # bucket. These peer lines help expose ambiguous or merged buckets.
            for peer_index, peer in enumerate(event_segments):
                if peer_index == segment_index:
                    continue
                geometry_overlap = len(
                    bucket_geometry_set & peer["geometry_set"]
                )
                if geometry_overlap == 0:
                    continue
                peer_tan_beta = (
                    None if peer["decoded"] is None
                    else peer["decoded"][0]
                )
                peer_y0 = (
                    None if peer["decoded"] is None
                    else peer["decoded"][1]
                )
                self.peer_segment_writer.writerow({
                    "event_id": event_id,
                    "target_segment_index": segment_index,
                    "source_bucket_id": source_bucket_id,
                    "peer_segment_index": peer_index,
                    "peer_true_tanBeta": (
                        "" if peer_tan_beta is None else peer_tan_beta
                    ),
                    "peer_true_y0": "" if peer_y0 is None else peer_y0,
                    "peer_n_geometry_ids": len(peer["geometry_set"]),
                    "peer_geometry_ids": ";".join(
                        f"0x{geometry_id:x}"
                        for geometry_id in sorted(peer["geometry_set"])
                    ),
                    "geometry_overlap": geometry_overlap,
                    "overlapping_geometry_ids": ";".join(
                        f"0x{geometry_id:x}"
                        for geometry_id in sorted(
                            bucket_geometry_set & peer["geometry_set"]
                        )
                    ),
                })


@njit(cache=True)
def classify_hits(y, z, radius, tan_beta, y0, tolerance):
    """Classify hits using |line-to-wire distance - drift radius|."""
    n_hits = len(y)
    mask = np.zeros(n_hits, dtype=np.uint8)
    residuals = np.empty(n_hits, dtype=np.float32)

    denominator = np.sqrt(1.0 + tan_beta * tan_beta)
    n_matching = 0
    residual_sum = 0.0

    for index in range(n_hits):
        distance = abs(y[index] - y0 - tan_beta * z[index]) / denominator
        residual = abs(distance - abs(radius[index]))

        residuals[index] = residual

        if residual <= tolerance:
            mask[index] = 1
            n_matching += 1
            residual_sum += residual

    mean_residual = (
        residual_sum / n_matching if n_matching > 0 else np.inf
    )

    return mask, residuals, n_matching, mean_residual


def as_numpy(array, dtype):
    """Convert an Awkward event array into a typed NumPy array."""
    return np.asarray(ak.to_numpy(array), dtype=dtype)


def decode_segment(parameters):
    """Decode stored local parameters into the Hough line (tanBeta, y0)."""
    parameters = as_numpy(parameters, np.float64)

    if parameters.size < 4:
        return None

    y0 = float(parameters[0])
    theta = np.deg2rad(float(parameters[1]))
    phi = np.deg2rad(float(parameters[3]))

    direction_z = np.cos(theta)

    if abs(direction_z) < 1.0e-12:
        return None

    tan_beta = float(np.sin(theta) * np.sin(phi) / direction_z)

    if not np.isfinite(tan_beta) or not np.isfinite(y0):
        return None

    return tan_beta, y0


def flush_hit_buffer(tree, buffers):
    """Append buffered hit columns to the output ROOT tree and release them."""
    if not buffers["event_id"]:
        return 0

    batch = {
        branch: np.concatenate(chunks)
        for branch, chunks in buffers.items()
    }

    rows = len(batch["event_id"])
    tree.extend(batch)

    for chunks in buffers.values():
        chunks.clear()

    return rows


def flush_truth_buffer(tree, buffers):
    """Append buffered fixed and variable-length truth columns to ROOT."""
    if not buffers["event_id"]:
        return 0

    batch = {
        branch: (
            ak.Array(chunks)
            if branch == "truth_hit_indices"
            else np.asarray(chunks, dtype=np.dtype(dtype))
        )
        for branch, dtype in TRUTH_OUTPUT_SCHEMA.items()
        for chunks in [buffers[branch]]
    }

    rows = len(batch["event_id"])
    tree.extend(batch)

    for chunks in buffers.values():
        chunks.clear()

    return rows


def main():
    parser = argparse.ArgumentParser(
        description="Create bucket-matched Eta Hough validation input."
    )

    parser.add_argument("input", type=Path)
    parser.add_argument(
        "output",
        type=Path,
        nargs="?",
        default=Path("EtaHoughFlatInput.root"),
    )
    parser.add_argument(
        "--truth-tolerance",
        type=float,
        default=5.0,
        help="Maximum |track-to-wire distance - drift radius| in mm.",
    )
    parser.add_argument(
        "--min-truth-hits",
        type=int,
        default=4,
        help="A segment matches a bucket when at least this many hits match.",
    )
    parser.add_argument("--max-events", type=int)
    parser.add_argument("--flush-rows", type=int, default=500_000)
    parser.add_argument(
        "--unmatched-prefix",
        type=Path,
        help=(
            "Prefix for unmatched segment, candidate, and hit CSV files. "
            "Defaults to <output stem>_unmatched."
        ),
    )
    parser.add_argument(
        "--unmatched-diagnostics",
        action="store_true",
        help="Do not write unmatched-segment diagnostic CSV files.",
    )

    args = parser.parse_args()

    unmatched_prefix = args.unmatched_prefix
    if unmatched_prefix is None:
        unmatched_prefix = args.output.with_name(
            args.output.stem + "_unmatched"
        )

    # 1. Read the two independent source ROOT trees. 
    with uproot.open(args.input) as input_file:
        space_tree = input_file["MuonSpacePoints"]
        truth_tree = input_file["MuonTruth"]

        number_events = space_tree.num_entries

        if args.max_events is not None:
            number_events = min(number_events, args.max_events)

        print(f"Reading {number_events} events...")

        space_data = space_tree.arrays(
            SPACE_BRANCHES,
            entry_stop=number_events,
            library="ak",
        )

        truth_data = truth_tree.arrays(
            TRUTH_BRANCHES,
            entry_stop=(number_events if args.max_events is not None else None),
            library="ak", # akward arrays coz diffrenet sizes
        )

    space_event_ids = as_numpy(
        space_data["event_id"], np.uint32
    )

    truth_event_ids = as_numpy(
        truth_data["event_id"], np.uint32
    )

    # 2. Join the trees through event_id
    truth_entry_by_event = {
        int(event_id): entry
        for entry, event_id in enumerate(truth_event_ids)
    }

    hit_buffers = {branch: [] for branch in OUTPUT_SCHEMA}
    truth_buffers = {branch: [] for branch in TRUTH_OUTPUT_SCHEMA}

    validation_bucket_id = 0
    validation_truth_id = 0
    buffered_rows = 0
    written_rows = 0
    written_truth_rows = 0

    matched_segments = 0
    unmatched_segments = 0
    unmatched_buckets = 0
    multi_segment_buckets = 0
    missing_truth_events = 0

    # 3. Create the optional CSV diagnostics and the two output ROOT trees.
    diagnostic_context = (
        nullcontext(None)
        if not args.unmatched_diagnostics
        else UnmatchedDiagnostics(
            unmatched_prefix, args.min_truth_hits
        )
    )

    with diagnostic_context as diagnostics, uproot.recreate(
        args.output
    ) as output_file:
        output_tree = output_file.mktree(
            "EtaValidationInput",
            OUTPUT_SCHEMA,
        )
        truth_output_tree = output_file.mktree(
            "EtaValidationTruth",
            TRUTH_OUTPUT_SCHEMA,
        )

        # 4. Process each event independently so bucket IDs remain event-local.
        for space_entry, event_id_value in enumerate(space_event_ids):
            event_id = int(event_id_value)
            truth_entry = truth_entry_by_event.get(event_id)

            if truth_entry is None:
                missing_truth_events += 1

            geometry_ids = as_numpy(
                space_data["spacePoint_geometryId"][space_entry],
                np.uint64,
            )
            bucket_ids = as_numpy(
                space_data["spacePoint_bucketId"][space_entry],
                np.uint16,
            )
            muon_ids = as_numpy(
                space_data["spacePoint_muonId"][space_entry],
                np.uint32,
            )

            local_x = as_numpy(
                space_data["spacePoint_localPosX"][space_entry],
                np.float32,
            )
            local_y = as_numpy(
                space_data["spacePoint_localPosY"][space_entry],
                np.float32,
            )
            local_z = as_numpy(
                space_data["spacePoint_localPosZ"][space_entry],
                np.float32,
            )

            sensor_phi = as_numpy(
                space_data["spacePoint_sensorDirPhi"][space_entry],
                np.float32,
            )
            sensor_theta = as_numpy(
                space_data["spacePoint_sensorDirTheta"][space_entry],
                np.float32,
            )
            next_phi = as_numpy(
                space_data["spacePoint_toNextDirPhi"][space_entry],
                np.float32,
            )
            next_theta = as_numpy(
                space_data["spacePoint_toNextDirTheta"][space_entry],
                np.float32,
            )

            drift_radius = np.abs(as_numpy(
                space_data["spacePoint_driftRadius"][space_entry],
                np.float32,
            ))
            hit_time = as_numpy(
                space_data["spacePoint_time"][space_entry],
                np.float32,
            )
            cov0 = as_numpy(
                space_data["spacePoint_covLoc0"][space_entry],
                np.float32,
            )
            cov1 = as_numpy(
                space_data["spacePoint_covLoc1"][space_entry],
                np.float32,
            )
            cov_t = as_numpy(
                space_data["spacePoint_covT"][space_entry],
                np.float32,
            )

            if truth_entry is None:
                segment_geometry_lists = []
                segment_parameter_lists = []
            else:
                segment_geometry_lists = (
                    truth_data["Segments_hitGeoIds"][truth_entry]
                )
                segment_parameter_lists = (
                    truth_data["Segments_localSegPars"][truth_entry]
                )

            # 5. Group the event's space points into physical input buckets and
            # cache each bucket's geometry-ID set for quick overlap tests.
            buckets = {}

            for source_bucket_id in np.unique(bucket_ids):
                indices = np.flatnonzero(
                    bucket_ids == source_bucket_id
                )

                buckets[int(source_bucket_id)] = {
                    "indices": indices,
                    "geometry_ids": set(map(
                        int,
                        np.unique(geometry_ids[indices]),
                    )),
                }

            segment_matches = []

            number_segments = max(
                len(segment_geometry_lists),
                len(segment_parameter_lists),
            )

            # 6. Decode every truth segment once and retain its geometry IDs.
            event_segments = []
            for segment_index in range(number_segments):
                has_parameters = (
                    segment_index < len(segment_parameter_lists)
                )
                has_geometry = (
                    segment_index < len(segment_geometry_lists)
                )

                if has_geometry:
                    segment_geometry_ids = np.unique(as_numpy(
                        segment_geometry_lists[segment_index],
                        np.uint64,
                    ))
                else:
                    segment_geometry_ids = np.empty(0, dtype=np.uint64)

                segment_geometry_set = set(map(
                    int, segment_geometry_ids
                ))

                decoded = (
                    decode_segment(
                        segment_parameter_lists[segment_index]
                    )
                    if has_parameters
                    else None
                )

                event_segments.append({
                    "has_parameters": has_parameters,
                    "has_geometry": has_geometry,
                    "geometry_ids": segment_geometry_ids,
                    "geometry_set": segment_geometry_set,
                    "decoded": decoded,
                })

            # 7. Match each usable segment to candidate buckets:
            #    a. require at least one shared geometry ID;
            #    b. classify every bucket hit against the truth drift circle;
            #    c. require at least --min-truth-hits compatible hits;
            #    d. keep only the highest-ranked bucket for that segment.
            for segment_index, segment in enumerate(event_segments):
                has_parameters = segment["has_parameters"]
                has_geometry = segment["has_geometry"]
                segment_geometry_ids = segment["geometry_ids"]
                segment_geometry_set = segment["geometry_set"]
                decoded = segment["decoded"]

                if decoded is None:
                    if diagnostics is not None:
                        diagnostics.write_segment(
                            event_id,
                            segment_index,
                            (
                                "invalid_parameters"
                                if has_parameters
                                else "missing_parameters"
                            ),
                            None,
                            None,
                            segment_geometry_set,
                            [],
                            {},
                            event_segments,
                        )
                    unmatched_segments += 1
                    continue

                tan_beta, y0 = decoded

                if segment_geometry_ids.size == 0:
                    if diagnostics is not None:
                        diagnostics.write_segment(
                            event_id,
                            segment_index,
                            (
                                "empty_geometry_ids"
                                if has_geometry
                                else "missing_geometry_ids"
                            ),
                            tan_beta,
                            y0,
                            segment_geometry_set,
                            [],
                            {},
                            event_segments,
                        )
                    unmatched_segments += 1
                    continue

                candidates = []
                geometry_candidates = []

                for source_bucket_id, bucket in buckets.items():
                    geometry_overlap = len(
                        segment_geometry_set
                        & bucket["geometry_ids"]
                    )

                    if geometry_overlap == 0:
                        continue

                    indices = bucket["indices"]

                    mask, residuals, n_matching, mean_residual = (
                        classify_hits(
                            local_y[indices],
                            local_z[indices],
                            drift_radius[indices],
                            tan_beta,
                            y0,
                            args.truth_tolerance,
                        )
                    )

                    candidate = {
                        "source_bucket_id": source_bucket_id,
                        "indices": indices,
                        "truth_mask": mask,
                        "residuals": residuals,
                        "n_truth_hits": n_matching,
                        "truth_fraction": n_matching / len(indices),
                        "geometry_overlap": geometry_overlap,
                        "mean_residual": mean_residual,
                        "segment_index": segment_index,
                        "tan_beta": tan_beta,
                        "y0": y0,
                    }
                    geometry_candidates.append(candidate)

                    if n_matching >= args.min_truth_hits:
                        candidates.append(candidate)

                if not candidates:
                    if diagnostics is not None:
                        diagnostics.write_segment(
                            event_id,
                            segment_index,
                            (
                                "no_geometry_overlap"
                                if not geometry_candidates
                                else "insufficient_compatible_hits"
                            ),
                            tan_beta,
                            y0,
                            segment_geometry_set,
                            geometry_candidates,
                            {
                                "geometry_id": geometry_ids,
                                "muon_id": muon_ids,
                                "local_x": local_x,
                                "local_y": local_y,
                                "local_z": local_z,
                                "drift_radius": drift_radius,
                                "cov1": cov1,
                            },
                            event_segments,
                        )
                    unmatched_segments += 1
                    continue

                # A segment may belong to only one bucket. Choose the bucket
                # with most compatible hits, then the smallest mean residual.
                candidates.sort(
                    key=candidate_sort_key,
                    reverse=True,
                )

                best = candidates[0]
                best["segment_index"] = segment_index
                best["tan_beta"] = tan_beta
                best["y0"] = y0

                segment_matches.append(best)
                matched_segments += 1

            # 8. Invert segment->bucket matches. A segment is unique to one
            # bucket, while one physical bucket may contain several segments.
            matches_per_bucket = Counter(
                match["source_bucket_id"]
                for match in segment_matches
            )

            multi_segment_buckets += sum(
                count > 1
                for count in matches_per_bucket.values()
            )

            matched_source_buckets = set(matches_per_bucket)
            unmatched_buckets += (
                len(buckets) - len(matched_source_buckets)
            )

            matches_by_bucket = {
                source_bucket_id: []
                for source_bucket_id in buckets
            }
            for match in segment_matches:
                matches_by_bucket[match["source_bucket_id"]].append(match)

            validation_id_by_source_bucket = {}

            # 9. Write every physical bucket exactly once, including buckets
            # without matched truth. Each hit becomes one ROOT tree entry.
            for source_bucket_id, bucket in sorted(buckets.items()):
                indices = bucket["indices"]
                number_hits = len(indices)
                selected_muon_ids = muon_ids[indices]
                bucket_matches = matches_by_bucket[source_bucket_id]

                def constant(value, dtype):
                    return np.full(number_hits, value, dtype=dtype)

                hit_buffers["validation_bucket_id"].append(
                    constant(validation_bucket_id, np.uint32)
                )
                hit_buffers["event_id"].append(
                    constant(event_id, np.uint32)
                )
                hit_buffers["source_bucket_id"].append(
                    constant(source_bucket_id, np.uint16)
                )
                hit_buffers["n_input_hits"].append(
                    constant(number_hits, np.uint32)
                )
                hit_buffers["n_truth_segments"].append(
                    constant(
                        len(bucket_matches),
                        np.uint16,
                    )
                )

                hit_buffers["geometry_id"].append(
                    geometry_ids[indices]
                )
                hit_buffers["muon_id"].append(
                    selected_muon_ids
                )
                hit_buffers["local_pos_x"].append(
                    local_x[indices]
                )
                hit_buffers["local_pos_y"].append(
                    local_y[indices]
                )
                hit_buffers["local_pos_z"].append(
                    local_z[indices]
                )
                hit_buffers["sensor_dir_phi"].append(
                    sensor_phi[indices]
                )
                hit_buffers["sensor_dir_theta"].append(
                    sensor_theta[indices]
                )
                hit_buffers["to_next_dir_phi"].append(
                    next_phi[indices]
                )
                hit_buffers["to_next_dir_theta"].append(
                    next_theta[indices]
                )
                hit_buffers["drift_radius"].append(
                    drift_radius[indices]
                )
                hit_buffers["time"].append(
                    hit_time[indices]
                )
                hit_buffers["cov_loc0"].append(
                    cov0[indices]
                )
                hit_buffers["cov_loc1"].append(
                    cov1[indices]
                )
                hit_buffers["cov_t"].append(
                    cov_t[indices]
                )

                validation_id_by_source_bucket[source_bucket_id] = (
                    validation_bucket_id
                )

                validation_bucket_id += 1
                buffered_rows += number_hits

                if buffered_rows >= args.flush_rows:
                    written_rows += flush_hit_buffer(
                        output_tree, hit_buffers
                    )
                    written_truth_rows += flush_truth_buffer(
                        truth_output_tree, truth_buffers
                    )
                    buffered_rows = 0

            # 10. Write one compact truth entry per accepted match. Hit indices
            # are local to the chosen bucket, which keeps the relation stable.
            for match in sorted(
                segment_matches,
                key=lambda value: (
                    value["source_bucket_id"],
                    value["segment_index"],
                ),
            ):
                truth_hit_indices = np.flatnonzero(
                    match["truth_mask"]
                ).astype(np.uint32, copy=False)

                truth_buffers["validation_truth_id"].append(
                    validation_truth_id
                )
                truth_buffers["validation_bucket_id"].append(
                    validation_id_by_source_bucket[
                        match["source_bucket_id"]
                    ]
                )
                truth_buffers["event_id"].append(event_id)
                truth_buffers["source_bucket_id"].append(
                    match["source_bucket_id"]
                )
                truth_buffers["segment_index"].append(
                    match["segment_index"]
                )
                truth_buffers["n_truth_hits"].append(
                    match["n_truth_hits"]
                )
                truth_buffers["true_tanBeta"].append(
                    match["tan_beta"]
                )
                truth_buffers["true_y0"].append(match["y0"])
                truth_buffers["truth_hit_indices"].append(
                    truth_hit_indices
                )
                validation_truth_id += 1

            if (space_entry + 1) % 1000 == 0:
                print(
                    f"Processed {space_entry + 1}/{number_events}, "
                    f"matched {matched_segments} segments, "
                    f"written {written_rows} rows"
                )

        written_rows += flush_hit_buffer(output_tree, hit_buffers)
        written_truth_rows += flush_truth_buffer(
            truth_output_tree, truth_buffers
        )

    if written_rows == 0:
        raise RuntimeError("No source buckets were written")

    print(f"Output: {args.output}")
    print(f"Matched truth segments: {matched_segments}")
    print(f"Unmatched truth segments: {unmatched_segments}")
    print(f"Source buckets without a segment: {unmatched_buckets}")
    print(f"Source buckets with multiple segments: {multi_segment_buckets}")
    print(f"Missing truth events: {missing_truth_events}")
    print(f"Validation buckets: {validation_bucket_id}")
    print(f"Validation truth segments: {validation_truth_id}")
    print(f"Written hit rows: {written_rows}")
    print(f"Written truth rows: {written_truth_rows}")
    if args.unmatched_diagnostics:
        print(f"Unmatched segment CSV: {unmatched_prefix}_segments.csv")
        print(f"Unmatched candidate CSV: {unmatched_prefix}_candidates.csv")
        print(f"Unmatched hit CSV: {unmatched_prefix}_hits.csv")
        print(
            "Unmatched peer-segment CSV: "
            f"{unmatched_prefix}_peer_segments.csv"
        )


if __name__ == "__main__":
    main()
