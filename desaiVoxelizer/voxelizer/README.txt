I wrote a simple main.cpp to demonstrate the usage.
To run:
./release/vox mesh.obj  [optional grid size]

It saves a file vox_out.txt
that should be self explanatory.

It does not handle internal cavities.
You have to do it by yourself:
-voxelize exterior and interior meshes
-subtract the interior voxels from the exterior voxels
