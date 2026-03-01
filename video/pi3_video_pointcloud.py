#
#  create pointcloud from video input file using pi3 ref:- https://github.com/yyfz/Pi3/tree/main
#
import torch
import argparse
from pi3.utils.basic import load_images_as_tensor, write_ply
from pi3.utils.basic import load_multimodal_data
from pi3.utils.geometry import depth_edge
from pi3.models.pi3 import Pi3
from pi3.models.pi3x import Pi3X
from pi3.pipe.pi3x_vo import Pi3XVO

if __name__ == '__main__':
    # --- Argument Parsing ---
    parser = argparse.ArgumentParser(description="Run inference with the Pi3 model.")
    
    parser.add_argument("--data_path", type=str, default='examples/skating.mp4',
                        help="Path to the input image directory or a video file.")
    parser.add_argument("--save_path", type=str, default='examples/result.ply',
                        help="Path to save the output .ply file.")
    parser.add_argument("--interval", type=int, default=-1,
                        help="Interval to sample image. Default: 1 for images dir, 10 for video")
    parser.add_argument("--ckpt", type=str, default=None,
                        help="Path to the model checkpoint file. Default: None")
    parser.add_argument("--device", type=str, default='cuda',
                        help="Device to run inference on ('cuda' or 'cpu'). Default: 'cuda'")
    parser.add_argument("--pipe", type=str, default='no',
                        help="use pipe yes or no Default: 'no'")
                        
    args = parser.parse_args()
    if args.interval < 0:
        args.interval = 10 if args.data_path.endswith('.mp4') else 1
    print(f'Sampling interval: {args.interval}')

    # 1. Prepare model
    if not args.pipe.find("yes") == -1:
        print(f"Loading model...")
        device = torch.device(args.device)
        if args.ckpt is not None:
            model = Pi3X().to(device).eval()
            if args.ckpt.endswith('.safetensors'):
                from safetensors.torch import load_file
                weight = load_file(args.ckpt)
            else:
                weight = torch.load(args.ckpt, map_location=device, weights_only=False)
            model.load_state_dict(weight, strict=False)
        else:
            model = Pi3X.from_pretrained("yyfz233/Pi3X").to(device).eval()
            # or download checkpoints from `https://huggingface.co/yyfz233/Pi3X/resolve/main/model.safetensors`, and `--ckpt ckpts/model.safetensors`
    else:	
        print(f"Loading model...")
        device = torch.device(args.device)
        if args.ckpt is not None:
            model = Pi3().to(device).eval()
            if args.ckpt.endswith('.safetensors'):
                from safetensors.torch import load_file
                weight = load_file(args.ckpt)
            else:
                weight = torch.load(args.ckpt, map_location=device, weights_only=False)
            model.load_state_dict(weight)
        else:
            model = Pi3.from_pretrained("yyfz233/Pi3").to(device).eval()
            # or download checkpoints from `https://huggingface.co/yyfz233/Pi3/resolve/main/model.safetensors`, and `--ckpt ckpts/model.safetensors`

    # 2. Prepare input data
    # The load_images_as_tensor function will print the loading path
    if not args.pipe.find("yes") == -1:
        imgs, _ = load_multimodal_data(args.data_path, conditions=None, interval=args.interval, device=device)
    else:
        imgs = load_images_as_tensor(args.data_path, interval=args.interval).to(device) # (N, 3, H, W)

    # 3. Infer
    print("Running model inference...")
    dtype = torch.bfloat16 if torch.cuda.get_device_capability()[0] >= 8 else torch.float16
    if not args.pipe.find("yes") == -1:
        pipe = Pi3XVO(model)
        with torch.no_grad():
            res = pipe(
                imgs=imgs, 
                dtype=dtype,
            )
    else:
        with torch.no_grad():
            with torch.amp.autocast('cuda', dtype=dtype):
                res = model(imgs[None]) # Add batch dimension

    # 4. process mask
    if not args.pipe.find("yes") == -1:
	    masks = res['conf'][0] > 0.05
    else:
        masks = torch.sigmoid(res['conf'][..., 0]) > 0.1
        non_edge = ~depth_edge(res['local_points'][..., 2], rtol=0.03)
        masks = torch.logical_and(masks, non_edge)[0]

    # 5. Save points
    print(f"Saving point cloud to: {args.save_path}")
    write_ply(res['points'][0][masks].cpu(), imgs.permute(0, 2, 3, 1)[masks], args.save_path)
    print("Done.")
