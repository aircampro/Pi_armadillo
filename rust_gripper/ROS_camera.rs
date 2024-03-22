// example from https://lib.rs/crates/opencv-ros-camera

use nalgebra::{Matrix2x3, Unit, Vector3};

// Here we have the YAML file contents hardcoded in a string. Ordinarily, you
// would read this from a file such as `~/.ros/camera_info/camera_name.yaml`, but
// for this example, it is hardcoded here.
let yaml_buf = b"image_width: 659
image_height: 494
camera_name: Basler_21029382
camera_matrix:
  rows: 3
  cols: 3
  data: [516.385667640757, 0, 339.167079537312, 0, 516.125799367807, 227.37993524141, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.331416226762003, 0.143584747015566, 0.00314558656668844, -0.00393597842852019, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [444.369750976562, 0, 337.107817516087, 0, 0, 474.186859130859, 225.062742824321, 0, 0, 0, 1, 0]";

// The ROS YAML file does not contain the pose (no extrinsic parameters). Here we
// specify them directly. The camera is at (10,0,0), looing at (0,0,0), with up (0,0,1).
let camcenter = Vector3::new(10.0, 0.0, 0.0);
let lookat = Vector3::new(0.0, 0.0, 0.0);
let up = Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0));
let pose = cam_geom::ExtrinsicParameters::from_view(&camcenter, &lookat, &up);

// We need the `serde-serialize` feature for the `from_ros_yaml` function.
#[cfg(feature = "serde-serialize")]
async fn main() {
    let from_ros = opencv_ros_camera::from_ros_yaml(&yaml_buf[..]).unwrap();

    // Finally, create camera from intrinsic and extrinsic parameters.
    let camera = cam_geom::Camera::new(from_ros.intrinsics, pose);
}