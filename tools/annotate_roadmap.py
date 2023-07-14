#!/usr/bin/env python3
import argparse
from multiprocessing import Pool
from typing import List, Tuple

import numpy as np
import yaml

N_PROCESSES = 8

if __name__ != "__main__":
    from . import collision


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing roadmap")
    parser.add_argument("out", help="output file containing annotated roadmap")
    parser.add_argument("radius", help="radius of robot",
                        type=float, default=0.3, nargs="?")
    args = parser.parse_args()
    print(args)

    with open(args.map) as map_file:
        roadmap = yaml.safe_load(map_file)

    if "roadmap" not in roadmap:
        print("Not a roadmap file!")
        exit()

    roadmap = add_self_edges(roadmap)
    roadmap = add_edge_conflicts(args.radius, roadmap)

    with open(args.out, 'w') as f:
        yaml.dump(roadmap, f)


def add_edge_conflicts(radius, roadmap):
    conflicts = compute_edge_conflicts(radius, roadmap)
    roadmap["roadmap"]["conflicts"] = conflicts
    return roadmap


def add_self_edges(roadmap):
    # if undirected, convert to a directed version
    if roadmap["roadmap"]["undirected"]:
        new_edges = [[goal, start]
                     for start, goal in roadmap["roadmap"]["edges"]]
        roadmap["roadmap"]["edges"].extend(new_edges)
        roadmap["roadmap"]["undirected"] = False

    # if wait actions are allowed, add self-edges
    if roadmap["roadmap"]["allow_wait_actions"]:
        new_edges = [[v, v] for v in roadmap["roadmap"]["vertices"]]
        roadmap["roadmap"]["edges"].extend(new_edges)
        roadmap["roadmap"]["allow_wait_actions"] = False

    return roadmap


def check_proxy(args):
    """Prox method to call the collision checker with the right arguments."""
    _, _, E, p0, p1, q0, q1 = args
    return collision.ellipsoid_collision_motion(E, p0, p1, q0, q1)


def compute_edge_conflicts(radius, map):
    # compute the pairwise collisions and add them to the map
    E = np.diag([radius, radius])
    num_edges = len(map["roadmap"]["edges"])
    v_dict = map["roadmap"]["vertices"]
    edges = map["roadmap"]["edges"]
    conflicts = [[] for _ in range(num_edges)]
    edges_to_check: List[Tuple[
        int, int, np.ndarray,  # i, j, E
        np.ndarray, np.ndarray, np.ndarray, np.ndarray  # p0, p1, q0, q1
    ]] = []
    for i in range(0, num_edges):
        p0 = np.asarray(v_dict[edges[i][0]])
        p1 = np.asarray(v_dict[edges[i][1]])
        for j in range(i+1, num_edges):
            if collision.precheck_indices(edges[i], edges[j]):
                # trivial case
                conflicts[i].append(j)
                conflicts[j].append(i)
            else:
                q0 = np.asarray(v_dict[edges[j][0]])
                q1 = np.asarray(v_dict[edges[j][1]])
                if collision.precheck_bounding_box(E, p0, p1, q0, q1):
                    edges_to_check.append((i, j, E, p0, p1, q0, q1))

    # check all edges in parallel

    with Pool(N_PROCESSES) as p:
        results = p.map(check_proxy, edges_to_check)
    for result, (i, j, _, _, _, _, _) in zip(results, edges_to_check):
        if result:
            conflicts[i].append(j)
            conflicts[j].append(i)

    return conflicts


if __name__ == "__main__":
    import collision
    main()
