# Example code for 3D matching: is taken from the example given as how to interface to Halcon
# https://github.com/SintefManufacturing/python-hcn?tab=readme-ov-file
#
import numpy as np
import hcn
import sys

if __name__ == "__main__":
    if len(sys.argv[0]) > 1:
        # first read CAD model and create surface model
		mod = hcn.Model3D.from_file(str(sys.argv[1]), "mm")
    else:
        mod = hcn.Model3D.from_file("CAD/KA1.STL", "mm")
    mod = mod.sampled("fast_compute_normals", 0.001)
    surf = mod.create_surface_model(0.001, invert_normals="true")

    #now read our scene
    if len(sys.argv[0]) > 2:
        scene = hcn.Model3D.from_file(str(sys.argv[2]), "mm")
    else:
        scene = hcn.Model3D.from_file("punktskyer/scene_ka1_simple.ply", "mm")
    scene = scene.compute_normals(60, 2)

    poses, score = surf.find_surface_model(scene, 0.001, 0.2, min_score=0, params={"num_matches":1})
    print("Found ", len(score), "matches: ", poses)