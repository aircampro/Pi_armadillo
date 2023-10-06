// node js to get connections in the database for a given file name
//
const [, , firstArg] = process.argv;

if (!firstArg) {
  console.error("Please pass the photo_file_name!!");
  process.exit(1);
}

const mysql = require('mysql');
var arr = [];
var my_file_name = process.argv[2];

// attempt to strip EOL chars however it passes <script> etc not the variable
// so this cant work in fixing the TBD in the php (perhaps try SQL in PHP)
//
//var ss="";
//var i = 0;
//while(i < my_file_name.length){
//    if (my_file_name[i] == ".") {
//        ss+=".jpg";
//        break;
//    } else {
//      ss+=my_file_name[i];
//      i++;
//    }
//    console.log(my_file_name[i]);
//}
//my_file_name=ss;
//console.log(my_file_name);
const connection = mysql.createConnection({
  host: '127.0.0.1',
  user: 'idm-client',
  password: 'acpAdman2350',
  database: 'InSpec'
});
var s = " ";
connection.connect((err) => {
  if (err) throw err;
});
connection.query('SELECT * FROM hotspot_image_link WHERE file_name = ? AND area_name = "Area A"', my_file_name, (err,rows) => {
  if(err) throw err;

  arr.push(rows.length);
});
connection.query('SELECT * FROM hotspot_image_link WHERE file_name = ? AND area_name = "Area B"', my_file_name, (err,rows) => {
  if(err) throw err;

  arr.push(rows.length);
});
connection.query('SELECT * FROM hotspot_image_link WHERE file_name = ? AND area_name = "Area C"', my_file_name, (err,rows) => {
  if(err) throw err;

  arr.push(rows.length);
});
connection.query('SELECT * FROM hotspot_image_link WHERE file_name = ? AND area_name = "Area D"', my_file_name, (err,rows) => {
  if(err) throw err;

  arr.push(rows.length);
  //console.log(arr);
  var first = 0;
  if (arr[0] >= 1) {
    s += "A";
    first = 1;
  }
  if (arr[1] >= 1) {
    if (first == 0) {
        s += "B";
        first = 1;
    } else {
        s += "-B";
    }
  }
  if (arr[2] >= 1) {
    if (first == 0) {
       s += "C";
       first = 1;
    } else {
       s += "-C";
    }
  }
  if (arr[3] >= 1) {
    if (first == 0) {
       s += "D";
       first = 1;
    } else {
       s += "-D";
    }
  }
  console.log(s);
});
connection.end();
