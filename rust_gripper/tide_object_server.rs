// webserver on rust using tide async-std and serde
// looks up or adds a new gripper object name and width in a mySQL database named gripper.db
//
// [dependencies]
// tide = "0.16.0"
// async-std = { version = "1.6.0", features = ["attributes"] }
// serde = { version = "1.0", features = ["derive"]
// [dependencies.rusqlite]
// version = "0.26.1"
// features = ["bundled"]
//  
use tide::Request;
//use tide::Response;
//use tide::Result;
//use tide::StatusCode;
use tide::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::collections::hash_map::{Entry};
use std::sync::{Arc, RwLock};

use std::io;
use rusqlite::{Connection, Result};

// =========== object to add to the database from a client request ============
//
#[derive(Serialize, Deserialize)]
struct ObjectSize {
    obj_name: String,
    width: f64,
}
impl ObjectSize {
    /// Create a new ObjectSize.
    fn new(obj_name: &str, width: Option<f64>) -> ObjectSize {
        ObjectSize { obj_name: obj_name.to_string(), width: width }
    }
    fn get_name(&self) -> &str {
        &self.obj_name
    }
    fn get_width(&self) -> f64 {
        self.width.unwrap_or(0) as f64
    }
}

// =========== object to find the width of in the database from a client request ============
//
#[derive(Debug, Deserialize)]
struct QueryObj {
    obj_name: String,
    userId: u16,
}
impl QueryObj {
    /// Create a new QueryObj
    fn new(obj_name: &str, userId: Option<u16>) -> QueryObj {
        QueryObj { obj_name: obj_name.to_string(), userId: userId }
    }
    fn get_name(&self) -> &str {
        &self.obj_name
    }
    fn get_id(&self) -> u16 {
        self.userId.unwrap_or(0) as u16
    }
}

#[async_std::main]
async fn main() -> Result<(), std::io::Error> {
//async fn main() -> Result<()> {
        tide::log::start();
        let mut app = tide::new();           

        // curl -i 'localhost:8080'
        // server replies with Gripper database server running! 
        //
        app.at("/").get(|_| async { Ok("Gripper database server running!") });
        // service client request to Add an object and its width to the database
        // curl -X POST -H "Content-Type: application/json" -d '{ "obj_name" : "medium_slab_4", "width" : 19.4 }' localhost:8080/ObjectSize
        // returns the ObjectSize names sent as a json message
        //
        app.at("/ObjectSize").post(|mut req: tide::Request<()>| async move {
            let ObjectSize: ObjectSize = req.body_json().await?;
			let conn = Connection::open("gripper.db")?;
            conn.execute(
                "create table if not exists obj_width 
                (obj_name VARCHAR(255), width FLOAT)",
            [],
            )?;
			conn.execute("INSERT INTO obj_width (obj_name, width) values (?1,?2)", [ObjectSize.get_name(), ObjectSize.get_width()])?;  
            Ok(tide::Body::from_json(&ObjectSize)?)
        });
        // client requests to get an objects width from the database
        // curl -X POST -H "Content-Type: application/json" -d '{ "obj_name" : "medium_slab_4", "width" : 19.4 }' localhost:8080/getwidth
        // returns the width as a value
        //
        app.at("/getwidth").post(|mut req: tide::Request<()>| async move {
            let obj_to_get: QueryObj = req.body_json().await?;
			if (obj_to_get.get_id() == 1 ) {
			    let conn = Connection::open("gripper.db")?;
                let query_answer = conn.execute(
                    "select width from obj_width where
                   (obj_name = (?1), [obj_to_get.get_name())",
                   [],
                )?;
			} else {
			  let query_answer = -1;              // invalid access key
			}
            Ok(query_answer)
        });
		// if we have more than one table for the objects e.g. floor 2 then these can apply
		//
        // service client request to Add an object and its width to the database
        // curl -X POST -H "Content-Type: application/json" -d '{ "name" : "medium_slab_4", "width" : 19.4 }' localhost:8080/ObjectSize2
        // returns the ObjectSize names sent as a json message
        //
        app.at("/ObjectSize2").post(|mut req: tide::Request<()>| async move {
            let ObjectSize: ObjectSize = req.body_json().await?;
			let conn = Connection::open("gripper.db")?;
            conn.execute(
                "create table if not exists obj_width2 
                (obj_name VARCHAR(255), width FLOAT)",
            [],
            )?;
			conn.execute("INSERT INTO obj_width2 (obj_name, width) values (?1,?2)", [ObjectSize.get_name(), ObjectSize.get_width()])?;  
            Ok(tide::Body::from_json(&ObjectSize)?)
        });
        // client requests to get an objects width from the database
        // curl -X POST -H "Content-Type: application/json" -d '{ "name" : "medium_slab_4", "width" : 19.4 }' localhost:8080/getwidth2
        // returns the width as a value
        //
        app.at("/getwidth2").post(|mut req: tide::Request<()>| async move {
            let obj_to_get: QueryObj = req.body_json().await?;
			if (obj_to_get.get_id() == 1 ) {
			    let conn = Connection::open("gripper.db")?;
                let query_answer = conn.execute(
                    "select width from obj_width2 where
                   (obj_name = (?1), [obj_to_get.get_name())",
                   [],
                )?;
			} else {
			  let query_answer = -1;              // invalid access key
			}
            Ok(query_answer)
        });
        app.listen("127.0.0.1:8080").await?;
        Ok(())
}

