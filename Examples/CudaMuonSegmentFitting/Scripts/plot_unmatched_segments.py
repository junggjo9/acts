# This file is part of the ACTS project.
#
# Copyright (C) 2016 CERN for the benefit of the ACTS project
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

"""Visualize unmatched Particle Gun truth segments and candidate buckets.

This is the diagnostic script for `preprocessor_pg.py`. It joins the four
CSV tables using `(event_id, segment_index)`, then creates one PNG per failed
truth segment. The drawing overlays the target truth line, all selected bucket
hits and drift circles, and any other muon truth lines sharing detector
geometry with those buckets.
"""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


def read_csv(path):
    """Yield CSV rows as dictionaries without loading the whole file at once."""
    with path.open(newline="") as input_file:
        yield from csv.DictReader(input_file)


def segment_key(row):
    """Return the shared key used by segment, candidate, and hit CSV rows."""
    return int(row["event_id"]), int(row["segment_index"])


def peer_target_key(row):
    """Return the target-segment key used by peer-segment CSV rows."""
    return int(row["event_id"]), int(row["target_segment_index"])


def optional_float(value):
    """Interpret an empty CSV cell as unavailable rather than as zero."""
    return None if value == "" else float(value)


def candidate_label(candidate):
    """Build a short legend label for one candidate bucket."""
    return (
        f"bucket {candidate['source_bucket_id']} "
        f"({candidate['n_compatible_hits']}/{candidate['n_input_hits']} "
        "compatible)"
    )


def draw_segment(
    segment, candidates, hits, peers, output_path, annotate_hits
):
    """Draw one failed truth segment and its possible bucket associations."""

    # 1. Use a large physics panel plus a text panel containing exact counts.
    figure, (axis, information) = plt.subplots(
        1,
        2,
        figsize=(16, 8),
        gridspec_kw={"width_ratios": [2.2, 1.0]},
    )

    event_id, segment_index = segment_key(segment)
    tan_beta = optional_float(segment["true_tanBeta"])
    y0 = optional_float(segment["true_y0"])

    # 2. Reconstruct candidate buckets from the flat hit CSV rows.
    hits_by_bucket = defaultdict(list)
    for hit in hits:
        hits_by_bucket[int(hit["source_bucket_id"])].append(hit)

    ordered_candidates = sorted(
        candidates, key=lambda row: int(row["candidate_rank"])
    )
    colour_map = plt.get_cmap("tab20")

    all_z = []
    all_y = []
    # 3. Draw each bucket in a separate colour. Crosses mark wire centres,
    # circles show drift radii, and open markers identify compatible hits.
    for candidate_index, candidate in enumerate(ordered_candidates):
        bucket_id = int(candidate["source_bucket_id"])
        bucket_hits = hits_by_bucket[bucket_id]
        colour = colour_map(candidate_index % 20)

        z_values = [float(hit["local_pos_z"]) for hit in bucket_hits]
        y_values = [float(hit["local_pos_y"]) for hit in bucket_hits]
        all_z.extend(z_values)
        all_y.extend(y_values)

        axis.scatter(
            z_values,
            y_values,
            s=32,
            marker="x",
            color=colour,
            alpha=0.85,
            label=candidate_label(candidate),
            zorder=3,
        )

        compatible_z = [
            float(hit["local_pos_z"])
            for hit in bucket_hits
            if int(hit["truth_compatible"]) != 0
        ]
        compatible_y = [
            float(hit["local_pos_y"])
            for hit in bucket_hits
            if int(hit["truth_compatible"]) != 0
        ]
        if compatible_z:
            axis.scatter(
                compatible_z,
                compatible_y,
                s=55,
                marker="o",
                facecolors="none",
                edgecolors=colour,
                linewidths=1.8,
                zorder=4,
            )

        for hit in bucket_hits:
            z = float(hit["local_pos_z"])
            y = float(hit["local_pos_y"])
            radius = abs(float(hit["drift_radius"]))
            if math.isfinite(radius) and radius > 0.0:
                axis.add_patch(
                    Circle(
                        (z, y),
                        radius,
                        fill=False,
                        color=colour,
                        linewidth=0.55,
                        alpha=0.28,
                        zorder=1,
                    )
                )

            if annotate_hits or int(hit["geometry_matches_segment"]) != 0:
                geometry_id = int(hit["geometry_id"])
                local_index = int(hit["bucket_local_hit_index"])
                axis.annotate(
                    f"h{local_index}\n0x{geometry_id:x}",
                    (z, y),
                    xytext=(3, 3),
                    textcoords="offset points",
                    fontsize=5.5,
                    alpha=0.72,
                )

    if all_z:
        z_min = min(all_z)
        z_max = max(all_z)
        z_padding = max(10.0, 0.08 * max(1.0, z_max - z_min))
    else:
        z_min, z_max, z_padding = -1000.0, 1000.0, 0.0
    line_z = [z_min - z_padding, z_max + z_padding]

    # 4. Overlay other muon truth segments that share geometry with a shown
    # bucket. Deduplicate peers appearing through more than one bucket.
    unique_peers = {}
    for peer in peers:
        peer_index = int(peer["peer_segment_index"])
        unique_peers.setdefault(peer_index, peer)
    for peer_index, peer in sorted(unique_peers.items()):
        peer_tan_beta = optional_float(peer["peer_true_tanBeta"])
        peer_y0 = optional_float(peer["peer_true_y0"])
        if peer_tan_beta is None or peer_y0 is None:
            continue
        peer_y = [peer_y0 + peer_tan_beta * z for z in line_z]
        axis.plot(
            line_z,
            peer_y,
            linestyle="--",
            linewidth=1.3,
            alpha=0.8,
            label=f"other muon truth segment {peer_index}",
            zorder=2,
        )

    # 5. Draw the failed target segment last so it is in front.
    if tan_beta is not None and y0 is not None:
        line_y = [y0 + tan_beta * z for z in line_z]
        axis.plot(
            line_z,
            line_y,
            color="black",
            linewidth=2.0,
            label="unmatched truth line",
            zorder=2,
        )

    axis.set_title(
        f"Event {event_id}, truth segment {segment_index}\n"
        f"failure: {segment['failure_reason']}"
    )
    axis.set_xlabel("local z [mm]")
    axis.set_ylabel("local y [mm]")
    axis.grid(True, alpha=0.2)
    axis.autoscale_view()
    if all_z:
        axis.set_aspect("equal", adjustable="datalim")
    if ordered_candidates or tan_beta is not None:
        axis.legend(loc="best", fontsize=7)

    # 6. Add an exact textual summary; detailed geometry IDs remain in CSV.
    information.axis("off")
    summary_lines = [
        "Unmatched truth summary",
        "",
        f"event: {event_id}",
        f"segment: {segment_index}",
        f"reason: {segment['failure_reason']}",
        f"tan(beta): {segment['true_tanBeta'] or 'unavailable'}",
        f"y0 [mm]: {segment['true_y0'] or 'unavailable'}",
        (
            "truth geometry IDs: "
            f"{segment['n_segment_geometry_ids']}"
        ),
        (
            "geometry-overlap buckets: "
            f"{segment['n_geometry_candidate_buckets']}"
        ),
        "",
        "rank  bucket  hits  geo  compatible  fraction  mean residual [mm]",
    ]
    for candidate in ordered_candidates[:30]:
        summary_lines.append(
            f"{int(candidate['candidate_rank']):>4}  "
            f"{int(candidate['source_bucket_id']):>6}  "
            f"{int(candidate['n_input_hits']):>4}  "
            f"{int(candidate['geometry_overlap']):>3}  "
            f"{int(candidate['n_compatible_hits']):>10}  "
            f"{float(candidate['truth_fraction']):>8.3f}  "
            f"{float(candidate['mean_residual_mm']):>18.3f}"
        )
    if len(ordered_candidates) > 30:
        summary_lines.append(
            f"... {len(ordered_candidates) - 30} more buckets in CSV"
        )
    if not ordered_candidates:
        summary_lines.append("No bucket shared a geometry identifier.")
    if unique_peers:
        summary_lines.extend(["", "Other geometry-overlapping muon truth:"])
        for peer_index, peer in sorted(unique_peers.items())[:20]:
            summary_lines.append(
                f"segment {peer_index}: tan(beta)="
                f"{peer['peer_true_tanBeta'] or 'unavailable'}, y0="
                f"{peer['peer_true_y0'] or 'unavailable'} mm"
            )

    information.text(
        0.0,
        1.0,
        "\n".join(summary_lines),
        va="top",
        ha="left",
        family="monospace",
        fontsize=8.2,
    )

    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Plot unmatched truth segments and all geometry-overlapping "
            "Particle Gun buckets."
        )
    )
    parser.add_argument(
        "prefix",
        type=Path,
        help=(
            "Diagnostic prefix produced by preprocessor_pg.py, for example "
            "EtaHoughFlatInput_unmatched."
        ),
    )
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--event", type=int)
    parser.add_argument("--segment", type=int)
    parser.add_argument(
        "--max-segments",
        type=int,
        default=100,
        help="Maximum number of plots; use 0 to plot every segment.",
    )
    parser.add_argument(
        "--top-buckets",
        type=int,
        default=0,
        help="Plot only the best N candidate buckets; 0 keeps all.",
    )
    parser.add_argument(
        "--annotate-hits",
        action="store_true",
        help="Annotate every hit rather than only geometry-matching hits.",
    )
    args = parser.parse_args()

    segment_path = args.prefix.with_name(
        args.prefix.name + "_segments.csv"
    )
    candidate_path = args.prefix.with_name(
        args.prefix.name + "_candidates.csv"
    )
    hit_path = args.prefix.with_name(args.prefix.name + "_hits.csv")
    peer_path = args.prefix.with_name(
        args.prefix.name + "_peer_segments.csv"
    )
    output_dir = args.output_dir or args.prefix.with_name(
        args.prefix.name + "_plots"
    )

    # 1. Select target segments first. This prevents loading hit rows for plots
    # that the event/segment/max-segments filters will discard.
    segments = []
    for row in read_csv(segment_path):
        event_id, segment_index = segment_key(row)
        if args.event is not None and event_id != args.event:
            continue
        if args.segment is not None and segment_index != args.segment:
            continue
        segments.append(row)

    if args.max_segments > 0:
        segments = segments[:args.max_segments]

    selected_keys = {segment_key(row) for row in segments}
    # 2. Join candidate buckets to the selected targets and optionally retain
    # only the best-ranked buckets for a less crowded visualization.
    candidates_by_segment = defaultdict(list)
    selected_buckets = defaultdict(set)
    for row in read_csv(candidate_path):
        key = segment_key(row)
        if key not in selected_keys:
            continue
        if (
            args.top_buckets > 0
            and int(row["candidate_rank"]) >= args.top_buckets
        ):
            continue
        candidates_by_segment[key].append(row)
        selected_buckets[key].add(int(row["source_bucket_id"]))

    # 3. Join only hits belonging to candidate buckets selected above.
    hits_by_segment = defaultdict(list)
    for row in read_csv(hit_path):
        key = segment_key(row)
        if key not in selected_keys:
            continue
        if int(row["source_bucket_id"]) not in selected_buckets[key]:
            continue
        hits_by_segment[key].append(row)

    # 4. Join other geometry-overlapping muon truth segments when available.
    peers_by_segment = defaultdict(list)
    if peer_path.exists():
        for row in read_csv(peer_path):
            key = peer_target_key(row)
            if key not in selected_keys:
                continue
            if int(row["source_bucket_id"]) not in selected_buckets[key]:
                continue
            peers_by_segment[key].append(row)

    # 5. Produce one independent PNG for every selected unmatched segment.
    output_dir.mkdir(parents=True, exist_ok=True)
    for plot_number, segment in enumerate(segments, start=1):
        key = segment_key(segment)
        event_id, segment_index = key
        output_path = output_dir / (
            f"event_{event_id:06d}_segment_{segment_index:04d}.png"
        )
        draw_segment(
            segment,
            candidates_by_segment[key],
            hits_by_segment[key],
            peers_by_segment[key],
            output_path,
            args.annotate_hits,
        )
        print(f"[{plot_number}/{len(segments)}] {output_path}")

    print(f"Wrote {len(segments)} plots to {output_dir}")


if __name__ == "__main__":
    main()
