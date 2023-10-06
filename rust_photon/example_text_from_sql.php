<html>
<head>
<script>
window.onpageshow = function(event) {
    if (event.persisted) {
         window.location.reload();
    }
};
window.addEventListener('pageshow',()=>{
    if(window.performance.navigation.type==2) location.reload();
});
</script>
</head>
<body>

 <center>
   <p id='msg2'>A-B-C-D</p>
   <p id='msg3'>A-B-C-D</p>
   <input type="checkbox" name="bbb1[1]" value="../thumbnails/AbbeyRoof (101)_s.jpg" id="btn" checked="unchecked"/>Picture 2
 </center>


<script>
   var my_file_name = document.getElementById('btn').value;
   console.log(my_file_name);
   var msg_p2=document.getElementById("msg2");
   msg_p2.innerHTML=my_file_name;
</script>

<?php
   $testV = "<script>document.writeln(my_file_name);</script>";
   // debug print...
   //
   //print($testV);
   // the variable above has something on the end which means it doesnt work
   // therefore only way for search to work is to hardcode it here
   // TBD :: find a better method than this
   //
   $file_id="../thumbnails/AbbeyRoof (101)_s.jpg";
   $command="node get_connection_node.js  \"";
   $command.=$file_id;
   $command.="\"";
   // debug print
   //
   //print("<br >");
   //print($command);
   exec($command,$opt,$status);
   //print("<br >");
   //print_r($opt);
?>

<script>
   // var jsvar = '<?=$var?>';
   <?php
       echo "var jsvar2 ='$opt[0]';";
   ?>
   var msg_p3=document.getElementById("msg3");
   msg_p3.innerHTML=jsvar2;
</script>
</body>
</html>
