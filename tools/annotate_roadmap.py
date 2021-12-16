#!/usr/bin/env python3
import argparse

import numpy as np
import yaml

import collision


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing roadmap")
    parser.add_argument("out", help="output file containing annotated roadmap")
    parser.add_argument("radius", help="radius of robot",
                        type=float, default=0.3, nargs="?")
    args = parser.parse_args()
    print(args)

    with open(args.map) as map_file:
        map = yaml.safe_load(map_file)

    if "roadmap" not in map:
        print("Not a roadmap file!")
        exit()

    # if undirected, convert to a directed version
    if map["roadmap"]["undirected"]:
        new_edges = [[goal, start] for start, goal in map["roadmap"]["edges"]]
        map["roadmap"]["edges"].extend(new_edges)
        map["roadmap"]["undirected"] = False

    # if wait actions are allowed, add self-edges
    if map["roadmap"]["allow_wait_actions"]:
        new_edges = [[v, v] for v in map["roadmap"]["vertices"]]
        map["roadmap"]["edges"].extend(new_edges)
        map["roadmap"]["allow_wait_actions"] = False

    # compute the pairwise collisions
    E = np.diag([args.radius, args.radius])
    num_edges = len(map["roadmap"]["edges"])
    v_dict = map["roadmap"]["vertices"]
    edges = map["roadmap"]["edges"]
    conflicts = [[] for _ in range(num_edges)]
    for i in range(0, num_edges):
        p0 = np.asarray(v_dict[edges[i][0]])
        p1 = np.asarray(v_dict[edges[i][1]])
        for j in range(i+1, num_edges):
            q0 = np.asarray(v_dict[edges[j][0]])
            q1 = np.asarray(v_dict[edges[j][1]])
            collides = collision.ellipsoid_collision_motion(E, p0, p1, q0, q1)
            if collides:
                conflicts[i].append(j)
                conflicts[j].append(i)
                print(edges[i], edges[j])

    print(conflicts)
    map["roadmap"]["conflicts"] = conflicts

    with open(args.out, 'w') as f:
        yaml.dump(map, f)


if __name__ == "__main__":
    main()
