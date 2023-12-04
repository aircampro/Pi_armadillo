// Rust Linux file server 
// In this example you can upload and download files make a log of last download and view it
// also you can invoke system commands and print the results as HTML
// added resnet on an uploaded image but still to test this out.................
//
// you must define the export folder before running
// e.g. EXPORT_FOLDER=/usr/nik/myfles; export EXPORT_FOLDER

// ===== Cargo.toml =====
// [dependencies]
// tide = "0.16.0"
// async-std = { version = "1.6.0", features = ["attributes"] }
// serde = { version = "1.0", features = ["derive"] }
// tempfile = "3.5.0"
// csv = "1.1"
// chrono = { version = "0.4", features = ["serde"] }
//

// add for resnet
// serde_json = "1.0"
// base64 = "0.13"
// image = "0.23"
// tch = "0.5.0"

// for running python library from rust
// Start a new project with cargo new and add pyo3 to the Cargo.toml like this:
// [dependencies.pyo3]
// version = "0.17.3"
// features = ["auto-initialize"]

// to run resnet we also need libtorch
//
//# install libtorch=1.9.0
//# https://pytorch.org/get-started/locally/
//WORKDIR /
//RUN wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.9.0%2Bcpu.zip -O libtorch.zip
//RUN unzip -o libtorch.zip
//ENV LIBTORCH /libtorch
//ENV LD_LIBRARY_PATH /libtorch/lib:$LD_LIBRARY_PATH
//
//# download weights file
//# https://github.com/LaurentMazare/tch-rs/blob/master/examples/pretrained-models/main.rs
//RUN wget https://github.com/LaurentMazare/ocaml-torch/releases/download/v0.1-unstable/resnet18.ot -O resnet18.ot
//WORKDIR /app

//extern crate image;
//extern crate tch;
//extern crate base64;

use std::io::Error as IoError;
use std::path::Path;
use std::sync::Arc;

use async_std::{fs::OpenOptions, io};
use tempfile::TempDir;
use tide::prelude::*;
use tide::{Body, Request, Response, StatusCode, Redirect};

use std::{env};
use std::fs::File;
use std::path::PathBuf;
use csv::Writer;

use chrono::{Utc, DateTime};

use std::process::Command;

// needed for resnet
//
//use serde::{Serialize, Deserialize};
//use serde_json::{json, Value};
//use std::sync::Arc;
//use tch::nn::ModuleT;
//use tch::vision::{resnet, imagenet};

// needed if you want to run a python library
// sudo apt install python3-dev
use pyo3::prelude::*;
use pyo3::types::IntoPyDict;

#[derive(Clone)]
struct TempDirState {
    tempdir: Arc<TempDir>,
}

impl TempDirState {
    fn try_new() -> Result<Self, IoError> {
        Ok(Self {
            tempdir: Arc::new(tempfile::tempdir()?),
        })
    }

    fn path(&self) -> &Path {
        self.tempdir.path()
    }
}

#[derive(Deserialize)]
struct FormSt {
 file1: String
}

//struct DnnModel {
//    net: Mutex<Box<dyn ModuleT>>
//}

#[async_std::main]
async fn main() -> Result<(), IoError> {
   
    let mut app = tide::with_state(TempDirState::try_new()?);
    app.with(tide::log::LogMiddleware::new());

//    resnet
//
//    let weights = std::path::Path::new("/resnet18.ot"); 
//    let mut vs = tch::nn::VarStore::new(tch::Device::Cpu);
//    let net:Mutex<Box<(dyn ModuleT + 'static)>> = Mutex::new(Box::new(resnet::resnet18(&vs.root(), imagenet::CLASS_COUNT)));
//    let _ = vs.load(weights);
//    let state = Arc::new(DnnModel { net });

    app.at(":file")
        // $ curl -T ./a.txt localhost:8080 # this writes the file to a temp directory
        //
        .put(|req: Request<TempDirState>| async move {
            let path = req.param("file")?;
            let p = req.param("file").unwrap().to_string();
            let fs_path = req.state().path().join(path);

            let file = OpenOptions::new()
                .create(true)
                .write(true)
                .open(&fs_path)
                .await?;

            let bytes_written = io::copy(req, file).await?;
            println!("file written bytes {}", bytes_written);
             let export_folder = env::var("EXPORT_FOLDER").map_err(|_| {
                 io::Error::new(io::ErrorKind::NotFound, "set env variable EXPORT_FOLDER to export path")
             })?;

             // write to a csv the file and time we downloaded it
             let file_path = PathBuf::from(export_folder).join("input.csv");
             
             let utc_datetime: DateTime<Utc> = Utc::now();
             println!("{}", utc_datetime); 

			 let file = File::create(&file_path)?;
             let mut writer = Writer::from_writer(file);
             let f_desc = "file uploaded was ".to_string();
             let text = Utc::now().format("%Y-%m-%d %H:%M:%S:%Z").to_string();
             let records = vec![
                 vec![&f_desc, &p, &text],
             ];
             for record in records {
                    writer.write_record(record)?;
             }  
             writer.flush()?;
             
            Ok(json!({ "bytes": bytes_written }))
        })
        // $ curl localhost:8080/a.txt # this reads the file from the same temp directory and logs it to file
        //
        .get(|req: Request<TempDirState>| async move {
             let path = req.param("file")?;
             let p = req.param("file").unwrap().to_string();
             let fs_path = req.state().path().join(path);
             
             let export_folder = env::var("EXPORT_FOLDER").map_err(|_| {
                 io::Error::new(io::ErrorKind::NotFound, "set env variable EXPORT_FOLDER to export path")
             })?;

             // write to a csv the file and time we downloaded it
             let file_path = PathBuf::from(export_folder).join("output.csv");
             
             let utc_datetime: DateTime<Utc> = Utc::now();
             println!("{}", utc_datetime); 
             
             if let Ok(body) = Body::from_file(fs_path).await {
			 let file = File::create(&file_path)?;
             let mut writer = Writer::from_writer(file);
             let f_desc = "file downloaded was ".to_string();
             let text = Utc::now().format("%Y-%m-%d %H:%M:%S:%Z").to_string();
             let records = vec![
                 vec![&f_desc, &p, &text],
             ];
             for record in records {
                 writer.write_record(record)?;
             }  
             writer.flush()?;

             Ok(body.into())
             } else {
                Ok(Response::new(StatusCode::NotFound))
             }
        });
        // do resnet on the files in memory 
        //
        // add a jpg like this :- $ curl -T ./a.jpg localhost:8080
        // curl localhost:8080/doresnet/:filename
        //   
        app.at("/doresnet/:filename").get(|_req: Request<TempDirState>| async move {
             let key: String = _req.param("filename").unwrap().to_string();
             let path = _req.param("filename")?;   
             let fs_path = _req.state().path().join(path);  
             let body = Body::from_file(fs_path).await;  
             //let net = state.net.lock().await;   
             //let img = image::load_from_memory(&body.as_slice()).unwrap(); 
             //let _ = img.save("/home/mark/rust/tmp.jpeg");
             //let img_tensor = imagenet::load_image_and_resize224("/home/mark/rust/tmp.jpeg").unwrap();
             //let img_tensor = imagenet::load_image_and_resize224(fs_path).unwrap();   <-- try this also.... example was like above
             //let output = net
             //   .forward_t(&img_tensor.unsqueeze(0), false)
             //   .softmax(-1, tch::Kind::Float);

             //let mut result = Vec::new();
             //for (probability, class) in imagenet::top(&output, 5).iter() {
             //    result.push(format!("{:50} {:5.2}%", class, 100.0 * probability).to_string());
             //}
             //Json(json!({ "result": result }))  
             //Ok(json!({ "result": result }))       
             
             // until built fully just return the file requested (then delete)                
             Ok(tide::Response::builder(200)
                 .body(format!("<html><h2>{} :-, !</h2></html>",key.to_string()))
                 .header("Server", "tide")
                 .content_type(tide::http::mime::HTML)
                 .build())
        });    
        // example of calling external python here for this example we analyse a dicom file in memory 
        //
        // add a dcm like this :- $ curl -T ./a.dcm localhost:8080
        // curl localhost:8080/dicom/:filename
        //   
        //app.at("/dicom/:filename").get(|_req: Request<TempDirState>| async move {
        //    let key: String = _req.param("filename").unwrap().to_string();
        //    let path = _req.param("filename")?;   
        //    let fs_path = _req.state().path().join(path);  
        //    let body = Body::from_file(fs_path).await;   // probably can change to use this but at present for testing path is hardcoded
		//	  let (patient_name, patient_address, patient_dob, user, os)  = call_python_libs(fs_path);
        //
        //    Ok(tide::Response::builder(200)
        //        .body(format!("<html><h2>Hello {} \n Address {} \n DOB {}, I'm Server {} os = {}</h2></html>", patient_name, patient_address, patient_dob, user, os))
        //        .header("Server", "tide")
        //        .content_type(tide::http::mime::HTML)
        //        .build())
        //}); 
        
        // curl localhost:8080/showlastdownload
        //       
        app.at("/showlastdownload").get(|_req: Request<TempDirState>| async move {
             let export_folder = env::var("EXPORT_FOLDER").map_err(|_| {
                 io::Error::new(io::ErrorKind::NotFound, "set env variable EXPORT_FOLDER to export path")
             })?;

             // write to a csv the file and time we downloaded it
             let file_path = PathBuf::from(export_folder).join("output.csv");
             let output = Command::new("/bin/cat")
                     //.arg("/home/mark/enfile/output.csv") <---- if you dont want to use the env variable
                     .arg(&file_path)
                     .output()
                     .expect("failed to execute process");
            println!("status: {}", output.status);
            println!("stdout: {}", String::from_utf8_lossy(&output.stdout));
            println!("stderr: {}", String::from_utf8_lossy(&output.stderr));
            Ok(tide::Response::builder(200)
                 .body(format!("<html><h2>Download csv :-, {}!</h2></html>",
                  String::from_utf8_lossy(&output.stdout)))
                 .header("Server", "tide")
                 .content_type(tide::http::mime::HTML)
                 .build())
        });   
        // curl localhost:8080/showlastupload
        //  
        app.at("/showlastupload").get(|_req: Request<TempDirState>| async move {
             let export_folder = env::var("EXPORT_FOLDER").map_err(|_| {
                 io::Error::new(io::ErrorKind::NotFound, "set env variable EXPORT_FOLDER to export path")
             })?;

             // write to a csv the file and time we downloaded it
             let file_path = PathBuf::from(export_folder).join("input.csv");
             let output = Command::new("/bin/cat")
                     //.arg("/home/mark/enfile/input.csv") <---- if you dont want to use the env variable
                     .arg(&file_path)
                     .output()
                     .expect("failed to execute process");
            println!("status: {}", output.status);
            println!("stdout: {}", String::from_utf8_lossy(&output.stdout));
            println!("stderr: {}", String::from_utf8_lossy(&output.stderr));
            Ok(tide::Response::builder(200)
                 .body(format!("<html><h2>Upload csv :-, {}!</h2></html>",
                  String::from_utf8_lossy(&output.stdout)))
                 .header("Server", "tide")
                 .content_type(tide::http::mime::HTML)
                 .build())
        }); 
        // shows the contents of the EXPORT_FOLDER variable
        // e.g. EXPORT_FOLDER=/usr/nik/myfles; export EXPORT_FOLDER
        // curl localhost:8080/showdirectory
        //   
        app.at("/showdirectory").get(|_req: Request<TempDirState>| async move {
			//let dname = req.param("dir").unwrap().to_string();
             let export_folder = env::var("EXPORT_FOLDER").map_err(|_| {
                 io::Error::new(io::ErrorKind::NotFound, "set env variable EXPORT_FOLDER to export path")
             })?;
            let output = Command::new("/bin/ls")
               .args(&["-l", "-a", &export_folder])
               .output()
               .expect("failed to start `ls`");
            println!("status: {}", output.status);
            println!("stdout: {}", String::from_utf8_lossy(&output.stdout));
            println!("stderr: {}", String::from_utf8_lossy(&output.stderr));
            Ok(tide::Response::builder(200)
                 .body(format!("<html><h2>Download csv :-, {}!</h2></html>",
                  String::from_utf8_lossy(&output.stdout)))
                 .header("Server", "tide")
                 .content_type(tide::http::mime::HTML)
                 .build())
        }); 
        // curl localhost:8080/rd
        // 
        app.at("/rd")
           .get(Redirect::new("https://www.youtube.com/watch?v=GGk_KrRyXLo"));
    app.listen("127.0.0.1:8080").await?;
    Ok(())
}

//async fn call_python_libs(filenm: &str) -> PyResult<[(String, String, String, String, String); N]> {		
//    Python::with_gil(|py| {
	    // get the os
//        let sys = py.import("os")?;
//        let os: String = os.name?.extract()?;

        // get the server username USER is linux while USERNAME is windows
//        let locals = [("os", py.import("os")?)].into_py_dict(py);
//        let code = "os.getenv('USER') or os.getenv('USERNAME') or 'Unknown'";
//        let user: String = py.eval(code, None, Some(&locals))?.extract()?;

        // check the dcm file for the patient needs pip install pydicom
//        let locals2 = [("pydicom", py.import("import pydicom")?)].into_py_dict(py);
//        let code_for_patient_name = "file = pydicom.dcmread('sample/my_dicom_file.dcm');print(file.[(0x0010, 0x0010)])";
//        let patient_name: String = py.eval(code_for_patient_name, None, Some(&locals2))?.extract()?;
//        let code_for_patient_address = "file = pydicom.dcmread('sample/my_dicom_file.dcm');print(file.[(0x0010, 0x1040)])";
//        let patient_address: String = py.eval(code_for_patient_address, None, Some(&locals2))?.extract()?;
//        let code_for_patient_dob = "file = pydicom.dcmread('sample/my_dicom_file.dcm');print(file.[(0x0010, 0x0030)])";
//        let patient_dob: String = py.eval(code_for_patient_dob, None, Some(&locals2))?.extract()?;
		
//        println!("Hello {} \n Address {} \n DOB {}, I'm Server {} os = {}", patient_name, patient_address, patient_dob, user, os);
//        let tuple: (String, String, String, String, String) = (patient_name, patient_address, patient_dob, user, os);
//        Ok(tuple)
//    })			
//}


