// example to connect to the database using node javascript and if the object is in it then disable the check box
//
const mysql = require('mysql');
var arr = [];
const connection = mysql.createConnection({
  host: '127.0.0.1',
  user: 'idm-client',
  password: 'acpAdman2350',
  database: 'InSpec'
});
var msg_p=document.getElementById("msg");
msg_p.innerHTML=" ";
var s = " ";
connection.connect((err) => {
  if (err) throw err;
});
var my_file_name = document.getElementById('btn').value;
connection.query('SELECT * FROM hotspot_image_link WHERE file_name = ? AND area_name = "Area A"', my_file_name, (err,rows) => {
  if(err) throw err;

  arr.push(rows.length);
  s += "A-";
});
connection.query('SELECT * FROM hotspot_image_link WHERE file_name = ? AND area_name = "Area B"', my_file_name, (err,rows) => {
  if(err) throw err;

  arr.push(rows.length);
  s += "B-";
});
connection.query('SELECT * FROM hotspot_image_link WHERE file_name = ? AND area_name = "Area C"', my_file_name, (err,rows) => {
  if(err) throw err;

  arr.push(rows.length);
  s += "C-";
});
connection.query('SELECT * FROM hotspot_image_link WHERE file_name = ? AND area_name = "Area D"', my_file_name, (err,rows) => {
  if(err) throw err;

  arr.push(rows.length);
  s += "D";
});
msg_p.innerHTML=s;
if (arr.length == 4) {
  if (arr[0] >= 1 && arr[1] >= 1 && arr[2] >= 1 && arr[3] >= 1) {
    document.getElementById('btn').disabled = true;
  } else {
    document.getElementById('btn').disabled = false;
  }
} else {
  document.getElementById('btn').disabled = false;
}
connection.end();
