#!/usr/bin/env python
#
# This demostrates the DA3 library it can also be used via linux command line
# ref:- https://github.com/ByteDance-Seed/Depth-Anything-3/tree/main?tab=readme-ov-file
# ref:- streaming https://github.com/ByteDance-Seed/Depth-Anything-3/tree/main/da3_streaming
#
import argparse
import glob, os, torch
from depth_anything_3.api import DepthAnything3

def main():
    parser = argparse.ArgumentParser(description="Process image files with DA3")
    parser.add_argument("--input_folder", type=str, default="./input", help="Path to png files")
    parser.add_argument("--vs", type=int, required=True, help="0=saddle 1=middle 2=saddle sim range")
    parser.add_argument("--output_folder", type=str, default="./output", help="output folder name")
    parser.add_argument("--export_format", type=int, required=True, help="0-4 export format")

    args = parser.parse_args()
    example_path = args.input_folder
    vs = args.vs
    output_path = args.output_folder
    exp_form = args.export_format
    vs_list = [ "saddle_balanced", "middle", "saddle_sim_range" ]
    if torch.cuda.is_available():
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")

    model = DepthAnything3.from_pretrained("depth-anything/DA3NESTED-GIANT-LARGE")
    model = model.to(device=device)
    images = sorted(glob.glob(os.path.join(example_path, "*.png")))
    if vs == 0:
        prediction = model.inference(images, ref_view_strategy="saddle_balanced")
    elif vs == 1:                                                      # For video sequences, consider using middle
        prediction = model.inference(images, ref_view_strategy="middle")
    elif vs == 2:                                                      # For complex scenes with wide baselines
        prediction = model.inference(images, ref_view_strategy="saddle_sim_range")
    # prediction.processed_images : [N, H, W, 3] uint8   array
    print(prediction.processed_images.shape)
    # prediction.depth            : [N, H, W]    float32 array
    print(prediction.depth.shape)  
    # prediction.conf             : [N, H, W]    float32 array
    print(prediction.conf.shape)  
    # prediction.extrinsics       : [N, 3, 4]    float32 array # opencv w2c or colmap format
    print(prediction.extrinsics.shape)
    # prediction.intrinsics       : [N, 3, 3]    float32 array
    print(prediction.intrinsics.shape)
    if exp_form == 0:                       # Export depth data and 3D visualization
        prediction = model.inference(image=images, ref_view_strategy=vs_list[vs], export_dir=output_path,  export_format="mini_npz-glb")
    elif exp_form == 1:                     # Export intermediate features from specific layers
        prediction = model.inference(image=images, ref_view_strategy=vs_list[vs], export_dir=output_path,  export_format="feat_vis", export_feat_layers=[0, 1, 2]  )     
    elif exp_form == 2:
        # Export multiple formats including Gaussian Splatting
        # Note: infer_gs=True requires da3-giant or da3nested-giant-large model
        if torch.cuda.is_available():
            model = DepthAnything3(model_name="da3-giant").to("cuda")
        else:
            model = DepthAnything3(model_name="da3-giant").to("cpu")
        if vs == 0:
            prediction = model.inference(images, ref_view_strategy="saddle_balanced")
        elif vs == 1:                                                      # For video sequences, consider using middle
            prediction = model.inference(images, ref_view_strategy="middle")
        elif vs == 2:                                                      # For complex scenes with wide baselines
            prediction = model.inference(images, ref_view_strategy="saddle_sim_range")			
        # Access depth maps
        depth_maps = prediction.depth  # shape: (2, H, W)
        # Access confidence
        if hasattr(prediction, 'conf'):
            confidence = prediction.conf
        # Access camera parameters (if available)
        if hasattr(prediction, 'extrinsics'):
            extrinsics_array = prediction.extrinsics  # shape: (2, 4, 4)
        if hasattr(prediction, 'intrinsics'):
            intrinsics_array = prediction.intrinsics  # shape: (2, 3, 3)
        # Access intermediate features (if export_feat_layers was set)
        if hasattr(prediction, 'aux') and 'feat_layer_0' in prediction.aux:
            features = prediction.aux['feat_layer_0']
        # You can export multiple formats simultaneously by separating them with -:
        # ref:- https://github.com/ByteDance-Seed/Depth-Anything-3/blob/main/docs/API.md
        #
        prediction = model.inference(
            image=images,
            ref_view_strategy=vs_list[vs],
            extrinsics=extrinsics_array,
            intrinsics=intrinsics_array,
            export_dir=output_path,
            export_format="npz-glb-gs_ply-gs_video",
            align_to_input_ext_scale=True,
            infer_gs=True,                                    # Required for gs_ply and gs_video exports
        )
    elif exp_form == 3:
        # Export with intermediate feature visualization
        prediction = model.inference(
            image=images,
            ref_view_strategy=vs_list[vs],
            export_dir=output_path,
            export_format="mini_npz-glb-depth_vis-feat_vis",
            export_feat_layers=[0, 5, 10, 15, 20],
            feat_vis_fps=30,
        )
    elif exp_form == 4:
        # Use ray-based pose estimation instead of camera decoder
        prediction = model.inference(
            image=images,
            ref_view_strategy=vs_list[vs],
            export_dir=output_path,
            export_format="glb",
            use_ray_pose=True,                                 # Enable ray-based pose estimation
        )
if __name__ == '__main__':
    main()


