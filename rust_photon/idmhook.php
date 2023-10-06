<?php
define('IN_PHPBB', true);                               // This tells the system that we are running as part of PHPBB
$phpbb_root_path = "/var/www/html/";
$forumPath="/var/www/html";                            // Check path.....

// Figure out our file extension and include some of the necessary libraries etc. Hook into the phpBB environment and initialise variables.
$phpEx = substr(strrchr(__FILE__, '.'), 1);
include ($phpbb_root_path . 'common.' . $phpEx);
include($phpbb_root_path . 'includes/functions_display.' . $phpEx);
$user->session_begin();
$auth->acl($user->data);
$Uname = $user->data['username'];
$UID = $user->data['user_id'];
$csPath="boltbox/";
$webpage='';
$Task = array();
$nowStamp=date('Y-m-d H:i:s');
$request->enable_super_globals();// temporarily enable superglobals to allow the use of $_POST
//----------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------
echo '<input type="text" name="example" list="exampleList">
<datalist id="exampleList">
    <option value="A">
    <option value="B">
    <option value="C">
    <option value="D">
    <option value="Test this one">
    <option value="One more to test">
</datalist>';

// Check entry conditions.
if (isset($_POST['TEST'])) {// You hit the TEST button. ******************************************* TEST ****************************************************
 $testing=test($_POST);
 echo $testing;
 $webpage.= $testing;
//----------------------------------------------------------------------------------------------------------------------------------------------

} else {
 $webpage.= globtest('murky', $testD);
 $webpage.= '...........................>>>'.$testC.'<<<.....................';
 $webpage.= '<BR>';
 $webpage.= 'mooooooooooooo';
}
//----------------------------------------------------------------------------------------------------------------------------------------------
// End of basic entry level checks.
//
//
//----------------------------------------------------------------------------------------------------------------------------------------------
//
// Subroutines.
//----------------------------------------------------------------------------------------------------------------------------------------------
//
//----------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------

function test($a) {//test routine
 $res=$a;
 $res[]="boobooboobooboobooboo";

 return $res;
}
//----------------------------------------------------------------------------------------------------------------------------------------------

function globtest($mkay, $testY) { // Perform global variables test.
 $response='';
 $response.='<BR><BR>';
 $response.='A-'.$GLOBALS['testA'].'--';
 $response.= '=:'.$mkay.':+:'.$testY.':=';
 $response.='C-'.$GLOBALS['testC'].'--';
 $GLOBALS['testC']='Was testC but not now.';
 return $response;
}
//----------------------------------------------------------------------------------------------------------------------------------------------

//function   { //Does What?
// $response='';
// return $response;
//}
//----------------------------------------------------------------------------------------------------------------------------------------------


//shell out of php and into html to assemble webpage to serve.
?>
 <html>
 <head>
 <style>
  body {background-color:black color:#6fe2da}
  border: 5px; border-style: dashed double
 </style>
 <title>Task Status</title>
<link rel="stylesheet" href="sample.css">
 </head>
 <body>

    <div class="disp-area">
      <div id="smallArea">
        <span>Upload</span>
      </div>
      <div id="largeArea">
        <span>To upload your files to the database <br/> Choose then click upload</span>
      </div>
    </div>
<span style="background:cyan; color:#7B68EE;" >
    <form action="upload.php" method="post" enctype="multipart/form-data">
        <input type="file" name="file"/>
        <input type="submit" name="submit" value="Upload"/>
    </form>
</span>
 <form method="post" action="<?php echo $PHP_SELF;?>">
 <font size="2">

<form action="php-bin/show.php" method="post">
  <p>This is a test</p>
  <input type="color" name="color" value="color">
  <input type="submit" name="submit" value="subMit" >
</form>

    <div class="disp-area">
      <div id="smallArea">
        <span>Area</span>
      </div>
      <div id="largeArea">
        <span>To look at an Area <br/> click radio for area then Click \"Choose Area\"</span>
      </div>
    </div>
    <br />
    <br />
    <fieldset>
    <legend>~~~~~~~~~~~ Look up photos related to this Area ~~~~~~~~~~~~~~~~</legend>
    <table>
<form method="post">
        <div>
        <p>Choose the area you want to query for pictures</p>
        <label><input type="radio" name="name2[]" value="Area A">Area A</label>
        <label><input type="radio" name="name2[]" value="Area B">Area B</label>
        <label><input type="radio" name="name2[]" value="Area C">Area C</label>
        <label><input type="radio" name="name2[]" value="Area D">Area D</label>
        <label><input type="radio" name="name2[]" value="Area E">Area E</label>
        <label><input type="radio" name="name2[]" value="Area F">Area F</label>
        <label><input type="radio" name="name2[]" value="Area G">Area G</label>
        <label><input type="radio" name="name2[]" value="Area H">Area H</label>
        <label><input type="radio" name="name2[]" value="Area I">Area I</label>
        </div>
        <input type="submit" value="Choose Area" >
</form>
</table>
</fieldset>

<?php
    // This php runs a python script calling openCV to re-size the image for small --- this will be called upon uplaod on each image
	//
    $command="python3 ./resize_image.py 2";
    exec($command,$output);
    print "$output[0]\n";
    print "$output[1]\n";
?>

<?php
    // Using MySQL Connecting, selecting database
    //
    //
    $link = mysqli_connect('127.0.0.1:3306', 'idm-client', 'acpAdman2350')
    or die('Could not connect to mySQL database: ' . mysql_error());
    echo 'Connected successfully to mySQL database';
    mysqli_select_db($link, "InSpec") or die('Could not select mySQL database InSpec');
?>

<?php
// Include the database configuration file
//
include 'dbConfig.php';

// List the images found in the database -- this is original code im not sure we need it anymore apart from maybe initially coloring the cards
// and then the table name has since changed.. as its what area each image is associated with
//
$query = $db->query("SELECT * FROM images ORDER BY uploaded_on DESC");
if($query->num_rows > 0){
    while($row = $query->fetch_assoc()){
        $imageURL = 'uploads/'.$row["file_name"];
?>
    <img src="<?php echo $imageURL; ?>" alt="" />
<?php }
}else{ ?>
    <p>No image(s) found...</p>

<?php } ?>


<?php //echo $webpage?>
</font>
</form>

<?php
    // mySQL free the query result executed
    mysqli_free_result($query);

    // Close out database mySQL DB Connections
    mysqli_close( $link );
    mysqli_close( $db );
?>

<?php
// ---- in these methods we a self contained we connect and then disconnect within the method
// example show all the areas assigned to pic No.2 (Hard Coded)
//
?>
<?php

$new_mysqli = new mysqli('127.0.0.1:3306', 'idm-client', 'acpAdman2350', 'InSpec');
if ($new_mysqli->connect_error) {
    echo $new_mysqli->connect_error;
    exit();
} else {
    $new_mysqli->set_charset("utf8");
}
$sql = 'SELECT area_name,pic_id FROM hotspot_image_link WHERE pic_id = 2';

// action the query
$result = $new_mysqli->query($sql);

print("The picture 2 spans the following areas <br />");
// print the query
foreach($result as $val) {
    echo $val['area_name'];
    echo $val['pic_id']. '<br />';
}
unset($val);
mysqli_free_result($result);
$new_mysqli->close();
?>

<?php
// show pic id and area name for a given area (specified as a variable)
//
$new_mysqli = new mysqli('127.0.0.1:3306', 'idm-client', 'acpAdman2350', 'InSpec');
if ($new_mysqli->connect_error) {
    echo $new_mysqli->connect_error;
    exit();
} else {
    $new_mysqli->set_charset("utf8");
}

//$sql = "SELECT pic_id, area_name, file_name FROM hotspot_image_link WHERE area_name = ? AND area_name = ?";
$sql = "SELECT pic_id, area_name, file_name FROM hotspot_image_link WHERE area_name = ?";
if ($stmt = $new_mysqli->prepare($sql)) {
    // Bind conditional values to SQL
    $area_name = "Area A";
    $sel_nam = $_POST['name2'][0];
    $area_name = $_POST['name2'][0];
    //$stmt->bind_param("is", $area_name, $area_name);
    $stmt->bind_param("s", $area_name);

    // execute the query
    $stmt->execute();

    // Bind the acquisition result to a variable
    //$stmt->bind_result($area_name, $area_name);
    $stmt->bind_result($pic_id, $area_name, $file_name);
    echo '<br />';
    print("<table border=\"1\" cellpadding=\"3\" cellspacing=\"1\" width=\"500\" bordercolor=\"#000000\">");
    print("<tr bgcolor=\"#e1f0ff\">");
    // uncomment results table if you want to look at it
    //
    //print("<table border=\"1\">");
    //print("<tr bgcolor=\"lightgreen\">");
    //print("<th>  Area Name  </th>");
    //print("<th>  Picture ID </th>");
    //print("<th>  Selection </th>");
    //print("<th>  File Name </th>");
    //printf("</tr>");
    while ($stmt->fetch()) {
        // uncomment results table if you want to look at it
        //
        //printf("<tr>");
        //printf("<td bgcolor=\"lightpink\">%s</td>", $area_name);
        //printf("<td>%s</td>", $pic_id);
        //printf("<td>%s</td>", $sel_nam);
        //printf("<td>%s</td>", $file_name);
        //print("</tr>");

        // old test code below
        //
        //$small_file=(string) $pic_id;
        //$large_file=(string) $pic_id;
        //$small_file.="_s.jpg";
        //$large_file.=".jpg";
        //printf("<a href=\"%s\" class=\"fancybox\" rel=\"gallery\" title=\"Click to enlarge\"><img src=\"%s\" /></a>",$large_file, $small_file);

        $small_file=$file_name;
        $large_file="intermediates/";
        $str2=$file_name;
        $arrX = preg_split("/[\/_.]+/",$str2);
        $c1=(string)count($arrX);
        //print_r($arrX);
        //printf(" RESULT == ../intermediates/%s.%s %s\n",$arrX[2],$arrX[(integer)$c1-1],$cl);
        if ((integer)$c1 >= 3) {
            $large_file.=$arrX[2];
            $large_file.=".";
            $large_file.=$arrX[(integer)$c1-1];
            $arrY = preg_split("/[.]+/",$str2);
            $c2=(string)count($arrY);
            $small_file=$arrY[1];
            $small_file.=".";
            $small_file.=$arrY[(integer)$c2-1];
            $arrZ = preg_split("/[\/]+/",$small_file);
            $c3=(string)count($arrZ);
            $small_file=$arrZ[(integer)$c3-2];
            $small_file.="/";
            $small_file.=$arrZ[(integer)$c3-1];
            //printf("file name == %s %s\n",$large_file,$small_file);
            print("<td align=\"center\">");
            printf("<img src=\"%s\" width=\"100\" height=\"100\" alt=\"imaG\">\n",$small_file);
            //print("</td>");
            // uncomment the fancybox below if you want it that way
            //
            printf("<a href=\"%s\" class=\"fancybox\" rel=\"gallery\" title=\"Click to enlarge\"><img src=\"%s\" /></a>",$large_file, $small_file);
            print("</td>");
       }
    }
    print("</tr>");
    print("</table>");
    // results table code
    //
    // echo '</table>';
    $stmt->close();

}
$new_mysqli->close();


print("<br /> <br />listing the files in /etc <br />");
$dir = "/etc/";

if (is_dir($dir)) {
    echo "<br />";
    if ($dh = opendir($dir)) {
        while (($file = readdir($dh)) !==  false) {
            //echo "filename: $file : filetype: " .  filetype($dir .  $file) .  "\n";
            echo "filename: $file : filetype: " .  filetype($dir .  $file) .  "<br />";
        }$new_mysqli->close();

        closedir($dh);
    }
}

$result = glob('./*');
echo '<br />';
var_dump($result);
echo '<br />';
$res = array( "123", "3445", "fjhds" );
$cnt=count($result);
$last = $cnt - 1;
echo "<br /> the first file is $result[0] third $result[2] last $result[$last] there are $cnt files <br />";
foreach($result as $i => $value){
  echo $i ." : ". $value. "<br />";
}
unset($i);
?>

<fieldset>
<legend>~~~~~~~~~~~ Look up an Areas photos ~~~~~~~~~~~~~~~~</legend>
<table>
<form name="frmContact2" method="post" action="area_lookup.php">
<p>
<label for="AreaName">Area Name: </label>
<input type="text" placeholder="The area name you want to look at" name="look_area" id="look_area">
</p>
<p>&nbsp;</p>
<p>
<input type="submit" name="Submit" id="Submit" value="Submit">
</p>
</form>
</table>
</fieldset>

<?php
// method to produce output page of all pictures in a specified given area
//
// uses fancybox
// sudo apt install npm
// npm install fancybox --save-dev
// sudo apt install libjs-jquery
//
// implementation using fancybox
// https://www.npmjs.com/package/fancybox
//
//
$new_mysqli = new mysqli('127.0.0.1:3306', 'idm-client', 'acpAdman2350', 'InSpec');
if ($new_mysqli->connect_error) {
    echo $new_mysqli->connect_error;
    exit();
} else {
    $new_mysqli->set_charset("utf8");
}

// use this for no value if you need it
$none = null;
$area_nam = "Area A";

$id_pic = 5;
$file_nam = "";
$small_file = "";

// get the area name from the screen form above - look for the photos in that area and display them
//
$area_nam = $_POST['look_area'];
printf("lok up area %s <br />",$area_nam);
$new_mysqli->close();
?>

</body></html>
