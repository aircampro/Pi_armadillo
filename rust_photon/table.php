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

?>