// example using Google Cloud API to get color information
// from information by https://gist.github.com/HirotoShioi
//
// [dependencies]
// reqwest = { version = "0.11.9", features = ["json", "blocking"] }
// tokio = { version = "1", features = ["full"] }
// serde = { version = "1.0", features = ["derive"] }
// serde_json = "1.0"
//
extern crate hex; // 0.4.0
extern crate reqwest; // 0.9.20
extern crate serde_json; // 1.0

use reqwest::{StatusCode, Url};
use serde_json::{json, Value};
use std::fmt;
use std::fs;

#[derive(Debug, Deserialize, Serialize)]
struct localObjAnnot {
    name: String,
    score: String,
//	bounding_poly: Vec<HashMap<f32, f32>> ---- dont need this data!
}

#[derive(Debug, Deserialize, Serialize)]
struct LabelObj {
    mid: String,
    description: String,
    score: f32,
    topicality: f32,
}

fn main() {
    // the picture to analyse is at this url it can your own using the rust tide fileserver in this directory
    let url =
        reqwest::Url::parse("http://www.ibukiyama-driveway.jp/images/flower/flower_20191006175111_02.jpg")
            .unwrap();
	
    // try IMAGE_PROPERTIES	
    let colors = get_dominant_colors(&url).unwrap();
    println!("{:#?}", colors);

    // try OBJECT_LOCALIZATION	
    let objs = get_dominant_category(&url).unwrap();
    println!("{:#?}", objs);

    // try LABEL_DETECTION		
    let lab, score = get_main_label(&url).unwrap();
    println!("main label found :: {} {}", lab, score);

    // try TEXT_DETECTION		
    let t1, t2 = get_main_text(&url).unwrap();
    println!("main label found :: {} {}", t1, t2);

    // try FACE_DETECTION	
    let face1 = get_main_face(&url).unwrap();
    println!("{}", face1);

    // try LANDMARK_DETECTION	
    let la, sc = get_main_landmark(&url).unwrap();
    println!("main landmark found :: {} {}", la, sc);
	
}

/// Use Google cloud vision API to extract list of dominant colors
pub fn get_dominant_colors(image_url: &Url) -> Result<Vec<Color>, Box<dyn std::error::Error>> {
    let response_json = use_cloud_vision_api(image_url)?;
    let extracted = extract_colors(&response_json)?;
    Ok(extracted)
}

/// Use Google cloud vision API to categorize the objects shown in the image as a vector output
pub fn get_dominant_category(image_url: &Url) -> Result<Vec<localObjAnnot>, Box<dyn std::error::Error>> {
    let response_json = use_cloud_localise_api(image_url)?;
    let extracted = extract_objects(&response_json)?;
    Ok(extracted)
}

/// Use Google cloud vision API to categorize the objects shown in the image as a vector output
pub fn get_dominant_labels(image_url: &Url) -> Result<Vec<LabelObj>, Box<dyn std::error::Error>> {
    let response_json = use_cloud_label_api(image_url)?;
    let extracted = extract_labels(&response_json)?;
    Ok(extracted)
}

/// Use Google cloud vision API to categorize the main object shown in the image
pub fn get_main_object(image_url: &Url) -> Result<(String, f32), Box<dyn std::error::Error>> {
    let response_json = use_cloud_localise_api(image_url)?;
    let main_obj, score = extract_main_object(&response_json)?;
    Ok((main_obj, score))
}

/// Use Google cloud vision API to label the main object shown in the image
pub fn get_main_label(image_url: &Url) -> Result<(String, f32), Box<dyn std::error::Error>> {
    let response_json = use_cloud_label_api(image_url)?;
    let main_label, score = extract_main_label(&response_json)?;
    Ok((main_label, score))
}

/// Use Google cloud vision API to use ocr to extract the text from the given image
pub fn get_main_text(image_url: &Url) -> Result<(String, String), Box<dyn std::error::Error>> {
    let response_json = use_cloud_text_api(image_url)?;
    let t1, t2 = extract_main_text(&response_json)?;
    Ok((t1, t2))
}

/// Use Google cloud vision API to read the facial expression of the face from the given image
pub fn get_main_face(image_url: &Url) -> Result<String, Box<dyn std::error::Error>> {
    let response_json = use_cloud_face_api(image_url)?;
    let face_info = extract_face(&response_json)?;
    Ok(face_info)
}

/// Use Google cloud vision API to describe landmarks shown in the given image
pub fn get_main_landmark(image_url: &Url) -> Result<(String, f32) Box<dyn std::error::Error>> {
    let response_json = use_cloud_landmark_api(image_url)?;
    let landmk, score = extract_main_landmark(&response_json)?;
    Ok((landmk, score))
}

const CLOUD_VISION_URI: &str = "https://vision.googleapis.com/v1/images:annotate";

//File path to a file which contains API key
const API_KEY_FILE_PATH: &str = "./secrets/vision_api.key";

///Use Google cloud vision api to parse given `image_url`
fn use_cloud_vision_api(image_url: &Url) -> Result<Value, Box<dyn std::error::Error>> {
    let request = json!({
        "requests": [
          {
            "image": {
                "source": {
                    "imageUri": image_url.to_owned().into_string()
                }
            },
            "features": [
              {
                "maxResults": 10,
                "type": "IMAGE_PROPERTIES"
              }
            ]
          }
        ]
    });

    let secret_key = fs::read_to_string(API_KEY_FILE_PATH)?;

    //--> this method is also possible uncomment below & replace .post(CLOUD_VISION_URI) with .post(&access_url)
    //let api_key: &str = "This is your API key";
    //let access_url = format!(
    //    "https://vision.googleapis.com/v1/images:annotate?key={}",
    //    api_key
    //);
	
    let mut response = reqwest::Client::new()
        .post(CLOUD_VISION_URI)
        .query(&[("key", secret_key)])
        .json(&request)
        .send()?;

    if response.status() != StatusCode::OK {
        return Err(Box::new(CloudVisionError::BadRequest));
    }

    let response_json: Value = response.json()?;

    let err = &response_json["responses"][0]["error"];

    if err.is_object() {
        return Err(Box::new(CloudVisionError::FailedToParseImage));
    }

    Ok(response_json)
}

///Use Google cloud vision api to parse given `image_url` for the object name
fn use_cloud_localise_api(image_url: &Url) -> Result<Value, Box<dyn std::error::Error>> {
    let request = json!({
        "requests": [
          {
            "image": {
                "source": {
                    "imageUri": image_url.to_owned().into_string()
                }
            },
            "features": [
              {
                "maxResults": 10,
                "type": "OBJECT_LOCALIZATION"
              }
            ]
          }
        ]
    });

    let secret_key = fs::read_to_string(API_KEY_FILE_PATH)?;

    let mut response = reqwest::Client::new()
        .post(CLOUD_VISION_URI)
        .query(&[("key", secret_key)])
        .json(&request)
        .send()?;

    if response.status() != StatusCode::OK {
        return Err(Box::new(CloudVisionError::BadRequest));
    }

    let response_json: Value = response.json()?;

    let err = &response_json["responses"][0]["error"];

    if err.is_object() {
        return Err(Box::new(CloudVisionError::FailedToParseImage));
    }

    Ok(response_json)
}

///Use Google cloud vision api for label detection with results set to first 2 lines `image_url` for the object name
fn use_cloud_label_api(image_url: &Url) -> Result<Value, Box<dyn std::error::Error>> {
    let request = json!({
        "requests": [
          {
            "image": {
                "source": {
                    "imageUri": image_url.to_owned().into_string()
                }
            },
            "features": [
              {
                "maxResults": 2,
                "type": "LABEL_DETECTION"
              }
            ]
          }
        ]
    });

    let secret_key = fs::read_to_string(API_KEY_FILE_PATH)?;

    let mut response = reqwest::Client::new()
        .post(CLOUD_VISION_URI)
        .query(&[("key", secret_key)])
        .json(&request)
        .send()?;

    if response.status() != StatusCode::OK {
        return Err(Box::new(CloudVisionError::BadRequest));
    }

    let response_json: Value = response.json()?;

    let err = &response_json["responses"][0]["error"];

    if err.is_object() {
        return Err(Box::new(CloudVisionError::FailedToParseImage));
    }

    Ok(response_json)
}

///Use Google cloud vision api for text detection 
fn use_cloud_text_api(image_url: &Url) -> Result<Value, Box<dyn std::error::Error>> {
    let request = json!({
        "requests": [
          {
            "image": {
                "source": {
                    "imageUri": image_url.to_owned().into_string()
                }
            },
            "features": [
              {
                "type": "TEXT_DETECTION"
              }
            ]
          }
        ]
    });

    let secret_key = fs::read_to_string(API_KEY_FILE_PATH)?;

    let mut response = reqwest::Client::new()
        .post(CLOUD_VISION_URI)
        .query(&[("key", secret_key)])
        .json(&request)
        .send()?;

    if response.status() != StatusCode::OK {
        return Err(Box::new(CloudVisionError::BadRequest));
    }

    let response_json: Value = response.json()?;

    let err = &response_json["responses"][0]["error"];

    if err.is_object() {
        return Err(Box::new(CloudVisionError::FailedToParseImage));
    }

    Ok(response_json)
}

///Use Google cloud vision api for landmark detection 
fn use_cloud_landmark_api(image_url: &Url) -> Result<Value, Box<dyn std::error::Error>> {
    let request = json!({
        "requests": [
          {
            "image": {
                "source": {
                    "imageUri": image_url.to_owned().into_string()
                }
            },
            "features": [
              {
                "maxResults": 2,
                "type": "LANDMARK_DETECTION"
              }
            ]
          }
        ]
    });

    let secret_key = fs::read_to_string(API_KEY_FILE_PATH)?;

    let mut response = reqwest::Client::new()
        .post(CLOUD_VISION_URI)
        .query(&[("key", secret_key)])
        .json(&request)
        .send()?;

    if response.status() != StatusCode::OK {
        return Err(Box::new(CloudVisionError::BadRequest));
    }

    let response_json: Value = response.json()?;

    let err = &response_json["responses"][0]["error"];

    if err.is_object() {
        return Err(Box::new(CloudVisionError::FailedToParseImage));
    }

    Ok(response_json)
}

///Use Google cloud vision api for face detection 
fn use_cloud_face_api(image_url: &Url) -> Result<Value, Box<dyn std::error::Error>> {
    let request = json!({
        "requests": [
          {
            "image": {
                "source": {
                    "imageUri": image_url.to_owned().into_string()
                }
            },
            "features": [
              {
                "maxResults": 2,
                "type": "FACE_DETECTION"
              }
            ]
          }
        ]
    });

    let secret_key = fs::read_to_string(API_KEY_FILE_PATH)?;

    let mut response = reqwest::Client::new()
        .post(CLOUD_VISION_URI)
        .query(&[("key", secret_key)])
        .json(&request)
        .send()?;

    if response.status() != StatusCode::OK {
        return Err(Box::new(CloudVisionError::BadRequest));
    }

    let response_json: Value = response.json()?;

    let err = &response_json["responses"][0]["error"];

    if err.is_object() {
        return Err(Box::new(CloudVisionError::FailedToParseImage));
    }

    Ok(response_json)
}

#[derive(Debug)]
pub struct Color {
    pub pixel_fraction: f32,
    pub score: f32,
    pub hex_color: String,
}

/// Extract list of `Color`s from given `val`
fn extract_colors(val: &Value) -> Result<Vec<Color>, CloudVisionError> {
    let colors = &val["responses"][0]["imagePropertiesAnnotation"]["dominantColors"]["colors"];

    match colors.as_array() {
        Some(color_ary) => {
            let mut color_vec = Vec::new();

            for color_value in color_ary.iter() {
                if let Some(color) = to_color(color_value) {
                    color_vec.push(color);
                } else {
                    return Err(CloudVisionError::UnableToParseColorData);
                };
            }
            Ok(color_vec)
        }

        None => Err(CloudVisionError::UnableToParseColorData),
    }
}

/// Extract list of Object catehgories from given `val`
fn extract_objects(val: &Value) -> Result<Vec<localObjAnnot>, CloudVisionError> {
    let obj_name = &val["responses"][0]["localizedObjectAnnotations"];
	
    match obj_name.as_array() {
        Some(obj_ary) => {
            let mut obj_vec = Vec::new();

            for value in obj_ary.iter() {
                if let Some(name) = to_object(value) {
                    obj_vec.push(name);
                } else {
                    return Err(CloudVisionError::UnableToParseColorData);
                };
            }
            Ok(obj_vec)
        }

        None => Err(CloudVisionError::UnableToParseColorData),
    }
}

/// Extract list of Object catehgories from given `val`
fn extract_labels(val: &Value) -> Result<Vec<LabelObj>, CloudVisionError> {
    let obj_name = &val["responses"][0]["labelAnnotations"];
	
    match obj_name.as_array() {
        Some(obj_ary) => {
            let mut obj_vec = Vec::new();

            for value in obj_ary.iter() {
                if let Some(name) = to_label(value) {
                    obj_vec.push(name);
                } else {
                    return Err(CloudVisionError::UnableToParseColorData);
                };
            }
            Ok(obj_vec)
        }

        None => Err(CloudVisionError::UnableToParseColorData),
    }
}

/// Extract the first Object catehgories from given `val`
fn extract_main_object(val: &Value) -> Result<(String, f32) CloudVisionError> {
    let obj_name = &val["responses"][0]["localizedObjectAnnotations"][0]["name"].to_string();
    let obj_score: f32 = &val["responses"][0]["localizedObjectAnnotations"][0]["score"];
    Ok((label, score))
}

/// Extract the label annotations from given `val`:Result<(String, String)>
fn extract_main_label(val: &Value) -> Result<(String, f32), CloudVisionError> {
    let label_name = &val["responses"][0]["labelAnnotations"][0]["description"].to_string();
    let label_score: f32 = &val["responses"][0]["labelAnnotations"][0]["score"];
    Ok((label, score))
}

/// Extract list of text from given `val`- didnt check yet dummy stub
fn extract_main_text(val: &Value) -> Result<(String, String), CloudVisionError> {
    let txt_full = &val["responses"][0]["fullTextAnnotation"]["text"].to_string();
    let obj_name = &val["responses"][0]["textAnnotations"]["text"].to_string();	
	Ok(txt_full, obj_name)
}

/// Extract facial expression from given `val``- didnt check yet dummy stub
fn extract_face(val: &Value) -> Result<String, CloudVisionError> {
    let mut txt_full = String::from("joy = ");
    txt_full.push_str(&val["responses"][0]["faceAnnotations"]["joyLikelihood"]);
    txt_full.push_str(" | sorrow = ");
    txt_full.push_str(&val["responses"][0]["faceAnnotations"]["sorrowLikelihood"]);
    txt_full.push_str(" | anger = ");
    txt_full.push_str(&val["responses"][0]["faceAnnotations"]["angerLikelihood"]);
    txt_full.push_str(" | surprise = ");	
    txt_full.push_str(&val["responses"][0]["faceAnnotations"]["surpriseLikelihood"]);
    txt_full.push_str(" | exposed = ");		
    txt_full.push_str(&val["responses"][0]["faceAnnotations"]["underExposedLikelihood"]);
    txt_full.push_str(" | blurred = ");		
    txt_full.push_str(&val["responses"][0]["faceAnnotations"]["blurredLikelihood"]);
    txt_full.push_str(" | headwaear = ");
    txt_full.push_str(&val["responses"][0]["faceAnnotations"]["headwearLikelihood"]);
    txt_full.push_str(" | ");
    Ok(txt_full)
}

/// Extract list of landmark from given `val``- didnt check yet dummy stub
fn extract_main_landmark(val: &Value) -> Result<(String, f32) CloudVisionError> {
    let txt_full = &val["responses"][0]["landmarkAnnotations"][0]["description"];
    let label_score: f32 = &val["responses"][0]["landmarkAnnotations"][0]["score"];
	Ok((txt_full, label_score))
}

///Construct `Color` struct with given `Value`
fn to_color(value: &Value) -> Option<Color> {
    let pixel_fraction = value.get("pixelFraction")?.as_f64()? as f32;
    let score = value.get("score")?.as_f64()? as f32;

    let color = &value.get("color")?;
    let red: u8 = color.get("red")?.to_owned().as_u64()? as u8;
    let green: u8 = color.get("green")?.to_owned().as_u64()? as u8;
    let blue: u8 = color.get("blue")?.to_owned().as_u64()? as u8;

    //Construct hex string color
    let mut hex_color = String::from("#");

    let hex = hex::encode(vec![red, green, blue]);

    hex_color.push_str(&hex);

    let color_struct = Color {
        pixel_fraction,
        score,
        hex_color,
    };

    Some(color_struct)
}

///Get the name and score from the local object annotation
fn to_object(value: &Value) -> Option<localObjAnnot> {
    let name = value.get("name")?.as_string()?;
    let score = value.get("score")?.as_f64()? as f32;

    let obj_struct = localObjAnnot {
        name,
        score,
    };

    // try this i think we can do this also
    // let obj_struct: localObjAnnot = serde_json::from_str(value)?;
    Some(obj_struct)
}

///Get the name and score from the local object annotation
fn to_label(value: &Value) -> Option<LabelObj> {
    let mid = value.get("mid")?.as_string()?;
    let description = value.get("description")?.as_string()?;
    let score = value.get("score")?.as_f64()? as f32;
    let topicality = value.get("topicality")?.as_f64()? as f32;
	
    let obj_struct = LabelObj {
        mid,
        description,
	score,
        topicality,		
    };
		  
    // try this i think we can do this also and it should be faster
    // let obj_struct: LabelObj = serde_json::from_str(value)?;
    Some(obj_struct)
}

#[derive(Debug)]
enum CloudVisionError {
    BadRequest,
    UnableToParseColorData,
    FailedToParseImage,
}

impl std::error::Error for CloudVisionError {}

impl fmt::Display for CloudVisionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> fmt::Result {
        let error_message = match self {
            CloudVisionError::BadRequest => "Bad request",
            CloudVisionError::FailedToParseImage => "Cloud vision api failed to parse image",
            CloudVisionError::UnableToParseColorData => "Unable to parse data",
        };
        write!(f, "{}", error_message)
    }
}
