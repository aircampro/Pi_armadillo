// ===================================== HTTP Client =============================================
//                                         main.rs
//
// ---- cargo.toml ---
//
// [dependencies]
// reqwest = { version = "0.11.9", features = ["json", "blocking"] }
// text-colorizer = "1"
// regex = "1"
// proconio = "0.3.6"
// tokio = { version = "1", features = ["full"] }
// serde = { version = "1.0", features = ["derive"] }
// serde_json = "1.0"
// ansi_term = "0.12.1"
// configparser = "3.0.0"
// async-std = { version = "1.7", features = ["attributes", "tokio1", "unstable"] }
//
use std::time::Instant;
use async_std::task;
use text_colorizer::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Deserialize, Serialize)]
struct authResponse {
    data: Vec<String>,
}

#[derive(Debug, Serialize, Deserialize)]
struct Post_json {
    id: Option<i32>,
    title: String,
    body: String,
    #[serde(rename = "userId")]
    user_id: i32,
}

// GET request blocking
fn get_request(url: &str) -> std::io::Result<reqwest::blocking::Response> {

    let client = reqwest::blocking::Client::new();
    let response = client.get(url)
                         .send().unwrap();

    println!("Access {}", response.url());
	
    println!("Response: {:?} {}", res.version(), res.status());
    println!("Headers: {:#?}\n", res.headers());
    let body = res.text().await?;
    println!("Body: {}", body);
	
    Ok(response)	
}

// GET simple
async fn getawait_request(url: &str) -> std::io::Result<reqwest::Response> {

    let client = reqwest::Client::new();
    let response = client.get(url)
                         .send().await.unwrap();

    println!("Access {}", response.url());
    Ok(response)
}

// GET json as hashmap
async fn getjson_request(url: &str) -> std::io::Result<HashMap<String, String>> {

    let resp: HashMap<String, String> = reqwest::get(url)?.json()?;
    println!("{:#?}", resp);
    Ok(resp)
}	

// POST a text
async fn post_txt_request(url: &str, body: &str) -> std::io::Result<()> {
    let client = reqwest::Client::new();
    let res = client.post(url)
        .body(body)
        .send()?;
		
    println!("Response: {:?} {}", res.version(), res.status());
    println!("Headers: {:#?}\n", res.headers());
    let body = res.text().await?;
    println!("Body: {}", body);
	
    Ok(())
}

// POST(Form)
async fn post_form_request(url: &str, s1: &str, s2 : &str, s3: &str, s4 : &str) -> std::io::Result<()> {
    let params = [(s1, s2), (s3, s4)];
    let client = reqwest::Client::new();
    let res = client.post(url)
        .form(&params)
        .send()?;
    Ok(())
}	

// JSON
async fn post_json_request(url: &str, v1: &str, v2: &str) -> std::io::Result<serde_json::Value> {
    let echo_json: serde_json::Value = reqwest::Client::new()
        .post(url)
        .json(&serde_json::json!({
            "item1": v1,
            "item2": v2,
            "userId": 1
        }))
        .send()
        .await?
        .json()
        .await?;

    println!("{:#?}", echo_json);
    Ok(echo_json)
}

async fn call_json2_request(url: &str) -> std::io::Result<()> {
    let new_post = Post_json {
        id: None,
        title: "Reqwest.rs".into(),
        body: "https://docs.rs/reqwest".into(),
        user_id: 1,
    };
	post_json2_request(url, &new_post);
    Ok(())
}

async fn call_spotify_artist_request(url: &str) -> std::io::Result<()> {
    let url = format!(
        "https://api.spotify.com/v1/search?q={query}&type=track,artist",
        // go check out her latest album. It's 🔥
        query = "Little Simz"
    );
    // the rest is the same as before!
    let client = reqwest::Client::new();
    let response = client
        .get(url)
        .header(AUTHORIZATION, "Bearer [AUTH_TOKEN]")
        .header(CONTENT_TYPE, "application/json")
        .header(ACCEPT, "application/json")
        .send()
       .await
       .unwrap();
        println!("Success! {:?}", response)
    Ok(())
}

async fn get_jpg_file_request(url: &str) -> std::io::Result<()> {
    let url = "https://pbs.twimg.com/profile_images/1058802892415455233/_Fat5vje_400x400.jpg";
    let filename = url.split("/").last().unwrap();
    let response: reqwest::Response = reqwest::get(url).await?;
    let bytes = response.bytes().await?;
    let mut out = File::create(filename)?;
    io::copy(&mut bytes.as_ref(), &mut out)?;

    Ok(())
}
   	
async fn post_json2_request(url: &str, new_post: &Post_json) -> std::io::Result<Post_json> {

    let new_rep: Post_json = reqwest::Client::new()
        .post(url)
        .json(new_post)
        .send()
        .await?
        .json()
        .await?;
    println!("{:#?}", new_rep);
    Ok(new_post)
}	
	
// Authorisation request
// 
const TOKEN: &str = "sampleToken123456";
async fn auth_request(url_specified: &str) -> std::io::Result<authResponse> {
    let bearer_token: &str = &format!("Bearer {}", TOKEN);
    let mut headers = header::HeaderMap::new();
    headers.insert("AUTHORIZATION", header::HeaderValue::from_str(bearer_token).unwrap());

    // http client
    let client = reqwest::Client::builder()
        .default_headers(headers)
        .build()?;

    let url = url_specified;
    let res = client
        .get(url)
        .send().await?
        .text().await
        .expect("データ取得の失敗");

    // json reply
    let res_json: authResponse = serde_json::from_str(&res).unwrap();
    Ok((res_json))	
}

pub async fn many_requests(requests: Vec<String>)
                           -> Vec<std::io::Result<reqwest::Response>> {

    let mut handles = vec![];
    for url in requests {
        handles.push(task::spawn_local(async move {
            getawait_request(&url).await
        }));
    }

    let mut results = vec![];
    for handle in handles {
        results.push(handle.await);
    }

    results
}

// The [cfg(not(target_arch = "wasm32"))] above prevent building the tokio::main function
// for wasm32 target, because tokio isn't compatible with wasm32.
// If you aren't building for wasm32, you don't need that line.
// The two lines below avoid the "'main' function not found" error when building for wasm32 target.
#[cfg(not(target_arch = "wasm32"))]
#[tokio::main]
fn main() -> Result<(), Box<std::error::Error>>) {

    // build a vector of the urls to test 
    let requests = vec![
    "http://localhost:8080/reset".to_string(),
    "http://localhost:8080/inc".to_string(),
    "http://localhost:8080/inc".to_string(),
    "http://localhost:8080/inc".to_string(),
    "http://localhost:8080/inc".to_string(),
    "http://localhost:8080/inc".to_string(),
    "http://localhost:8080/sub".to_string()
    ];

    // call get reqest function
    let start1 = Instant::now();
    for request in &requests {
        let result = get_request(request);
        match result {
            Ok(response) => {
                println!("{}", response.status());
            },
            Err(err) => {
                println!("NG");
                eprintln!("Err {}", err);
            },
        }
    }
    let end1 = start1.elapsed();
    # print results
    println!("{}: {}.{:03} [sec]", "Process time".blue().bold(), end1.as_secs(), end1.subsec_nanos() / 1_000_000);

    // call get many_requests function 
    let start2 = Instant::now();
    # now test with many requests
    let results = async_std::task::block_on(many_requests(requests));
     
    for result in &results {
        match result {
            Ok(response) => {
                println!("{}", response.status());
            },
            Err(err) => {
                println!("NG");
                eprintln!("Err {}", err);
            },
        }
    }
    let end2 = start2.elapsed();
    
    # print the results
    println!("{}: {}.{:03} [sec]", "Process time".blue().bold(), end2.as_secs(), end2.subsec_nanos() / 1_000_000);
}
